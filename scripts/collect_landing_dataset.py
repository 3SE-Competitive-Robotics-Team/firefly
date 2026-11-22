"""Curate CIF screenshots and recorded down-camera frames into a YOLO dataset.

Raw pixels come from the saved images/RRDs. CAD emission masks provide labels,
including occlusion, independently of the detector being evaluated. Blender
renders masks only; it never replaces the captured training image.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
RAW_SHA256 = "a3257397838e76cb130545d13cb77e8f612377af147af5deea5e6a38c2880bd0"


def digest(path):
    with Path(path).open("rb") as stream:
        return hashlib.file_digest(stream,"sha256").hexdigest()


def pixel_digest(pixels):
    return hashlib.sha256(pixels).hexdigest()


def pose_at(poses, stamp):
    """Exact GT pose, or interpolation across at most 50ms of recorded GT."""
    import bisect
    stamps = sorted(poses)
    i = bisect.bisect_left(stamps,stamp)
    if i<len(stamps) and stamps[i]==stamp:
        return poses[stamp],False
    if i==0 or i==len(stamps) or stamps[i]-stamps[i-1]>50_000_000:
        return None,False
    left,right=stamps[i-1:i+1]
    fraction=(stamp-left)/(right-left)
    a,b=poses[left],poses[right]
    position=np.array(a[:3])*(1-fraction)+np.array(b[:3])*fraction
    q1,q2=np.array(a[3:]),np.array(b[3:])
    q1/=np.linalg.norm(q1);q2/=np.linalg.norm(q2)
    dot=float(np.dot(q1,q2))
    if dot<0:q2=-q2;dot=-dot
    angle=math.acos(min(dot,1.))
    quaternion=q1*(1-fraction)+q2*fraction if angle<1e-6 else (
        q1*math.sin((1-fraction)*angle)+q2*math.sin(fraction*angle))/math.sin(angle)
    quaternion/=np.linalg.norm(quaternion)
    return [*position.tolist(),*quaternion.tolist()],True


def mask_labels(mask):
    """Visible complete-board solids; H paint must be present, as in se3_yolo."""
    height,width=mask.shape[:2]
    labels=[]
    for channel in (0,1):
        visible=mask[:,:,channel]>.5
        y,x=np.where(visible)
        if not len(x):continue
        w,h=x.max()-x.min()+1,y.max()-y.min()+1
        if min(w,h)<12 or np.count_nonzero(visible & (mask[:,:,2]>.5))<9:
            return None
        labels.append([0,(float(x.min())+float(x.max())+1)/(2*width),
                       (float(y.min())+float(y.max())+1)/(2*height),float(w)/width,float(h)/height])
    return labels


def render_masks(manifest):
    import bpy
    from mathutils import Quaternion,Vector
    metadata=json.loads(manifest.read_text())
    output=manifest.parent
    raw=ROOT/"models/rmuc2027/v1.0.0/derived/raw/field_raw.npz"
    if digest(raw)!=RAW_SHA256:raise ValueError("Review H part identities for changed CAD")
    data=np.load(raw)
    vertices,faces,parts,colors=(data[k] for k in ("vertices","faces","part","face_colors"))
    centers=vertices[faces].mean(axis=1)
    bpy.ops.wm.read_factory_settings(use_empty=True)
    mesh=bpy.data.meshes.new("semantic_field")
    mesh.vertices.add(len(vertices));mesh.vertices.foreach_set("co",vertices.ravel())
    mesh.loops.add(len(faces)*3);mesh.loops.foreach_set("vertex_index",faces.ravel())
    mesh.polygons.add(len(faces));mesh.polygons.foreach_set("loop_start",np.arange(len(faces),dtype=np.int32)*3)
    if "loop_total" in mesh.polygons[0].bl_rna.properties:
        mesh.polygons.foreach_set("loop_total",np.full(len(faces),3,dtype=np.int32))
    mesh.update()
    for index,color in enumerate(((0,0,0),(1,0,0),(0,1,0),(1,0,1),(0,1,1))):
        material=bpy.data.materials.new(f"semantic_{index}");material.use_nodes=True
        material.node_tree.nodes.clear()
        emission=material.node_tree.nodes.new("ShaderNodeEmission")
        emission.inputs["Color"].default_value=(*color,1.)
        terminal=material.node_tree.nodes.new("ShaderNodeOutputMaterial")
        material.node_tree.links.new(emission.outputs[0],terminal.inputs[0]);mesh.materials.append(material)
    indices=np.zeros(len(faces),dtype=np.int32)
    for index,sign in enumerate((-1,1)):
        board=(parts==10)&(centers[:,0]*sign>0)
        indices[board]=index+1;indices[board&(colors[:,:3].min(axis=1)>.95)]=index+3
    mesh.polygons.foreach_set("material_index",indices)
    obj=bpy.data.objects.new("field",mesh);bpy.context.collection.objects.link(obj)
    scene=bpy.context.scene;scene.render.engine="BLENDER_EEVEE";scene.eevee.taa_render_samples=4
    scene.render.resolution_x=352;scene.render.resolution_y=288;scene.render.resolution_percentage=100
    scene.view_settings.view_transform="Standard";scene.view_settings.exposure=0.;scene.view_settings.gamma=1.
    camera=bpy.data.objects.new("down",bpy.data.cameras.new("down"));bpy.context.collection.objects.link(camera)
    camera.data.sensor_fit="VERTICAL";camera.data.sensor_height=24
    camera.data.lens=24/(2*math.tan(math.radians(70.88/2)))
    camera.data.clip_start=.05;camera.data.clip_end=100;scene.camera=camera
    scene.render.image_settings.file_format="PNG";scene.render.image_settings.color_mode="RGB"
    scene.world=bpy.data.worlds.new("black");scene.world.color=(0,0,0)
    for frame in metadata["frames"]:
        if frame["body_pose"] is None:continue
        x,y,z,qx,qy,qz,qw=frame["body_pose"]
        body=Quaternion((qw,qx,qy,qz)).normalized()
        camera.location=Vector((x,y,z))+body@Vector((-.03,0,0))
        camera.rotation_mode="QUATERNION";camera.rotation_quaternion=body@Quaternion((0,0,1),-math.pi/2)
        mask_path=output/frame["mask"];mask_path.parent.mkdir(parents=True,exist_ok=True)
        scene.render.filepath=str(mask_path);bpy.ops.render.render(write_still=True)
        image=bpy.data.images.load(str(mask_path),check_existing=False)
        pixels=np.empty(352*288*4,dtype=np.float32);image.pixels.foreach_get(pixels);bpy.data.images.remove(image)
        labels=mask_labels(pixels.reshape(288,352,4)[::-1,:,:3])
        if labels is None:
            frame["label_status"]="ambiguous_visible_fragment"
            frame["split"]="quarantine"
        else:
            frame["label_status"]="cad_visibility_mask"
            frame["labels"]=labels;frame["negative"]=not labels
        label_path=output/frame["label"];label_path.parent.mkdir(parents=True,exist_ok=True)
        label_path.write_text("" if labels is None else "".join(str(int(row[0]))+" "+" ".join(f"{v:.8f}" for v in row[1:])+"\n" for row in labels))
        frame["mask_sha256"]=digest(mask_path);frame["label_sha256"]=digest(label_path)
    metadata["mask_renderer"]={"blender":bpy.app.version_string,"cad_sha256":RAW_SHA256}
    manifest.write_text(json.dumps(metadata,indent=2)+"\n")


def collect(args):
    from PIL import Image,ImageDraw
    from firefly_workspace.recording import recording_chunks, recording_files
    output=args.output.resolve();output.mkdir(parents=True,exist_ok=False)
    entries={}
    sources=sorted(args.images.rglob("*.png"))
    for path in sources:
        image=Image.open(path).convert("L")
        if image.size!=(352,288):raise ValueError(f"Expected CIF image: {path}")
        pixels=image.tobytes();key=pixel_digest(pixels)
        row=entries.setdefault(key,{"pixels":pixels,"screenshots":[],"sources":[]})
        row["screenshots"].append(str(path.resolve()))
    recordings=[]
    for path in sorted(args.recordings):
        poses={};matches=[];active=False;next_sample=0.;samples=0
        for chunk in recording_chunks(path):
            batch=chunk.to_record_batch()
            if "sim_time" not in batch.schema.names:continue
            stamps=batch.column("sim_time").cast("int64").to_pylist()
            if chunk.entity_path=="/gt/pose" and "Transform3D:translation" in batch.schema.names:
                for stamp,p,q in zip(stamps,batch.column("Transform3D:translation").to_pylist(),batch.column("Transform3D:quaternion").to_pylist()):
                    if p and q:poses[stamp]=[*p[0],*q[0]]
            elif chunk.entity_path=="/fc/debug/state" and "Scalars:scalars" in batch.schema.names:
                active|=any(row and row[0]!=0 for row in batch.column("Scalars:scalars").to_pylist())
            elif chunk.entity_path=="/cameras/down/image" and "Image:buffer" in batch.schema.names:
                for stamp,buffer in zip(stamps,batch.column("Image:buffer").to_pylist()):
                    if not buffer:continue
                    pixels=bytes(buffer[0])
                    if len(pixels)!=352*288:raise ValueError(f"Bad CIF frame in {path}")
                    key=pixel_digest(pixels)
                    sample=args.sample_period>0 and stamp/1e9>=next_sample and samples<args.max_samples
                    if sample:
                        next_sample=stamp/1e9+args.sample_period
                        sample=np.frombuffer(pixels,dtype=np.uint8).std()>=10
                        if sample:samples+=1
                    if key in entries or sample:
                        entries.setdefault(key,{"pixels":pixels,"screenshots":[],"sources":[]})
                        matches.append((key,stamp))
        group=(path.name if path.is_dir() else path.parent.name) if active else "fixed_poses"
        for key,stamp in matches:
            pose,interpolated=pose_at(poses,stamp)
            if pose is not None:entries[key]["sources"].append({"recording":str(path.resolve()),"timestamp_ns":stamp,
                "group":group,"body_pose":pose,"pose_interpolated":interpolated})
        with recording_files(path) as files:
            checksum = ({"segments": [{"name": f.name, "sha256": digest(f)} for f in files]}
                        if path.is_dir() else {"sha256": digest(path)})
        recordings.append({"path":str(path.resolve()),**checksum,"group":group})
    groups=sorted({source["group"] for row in entries.values() for source in row["sources"]})
    if len(groups)<3:raise ValueError("Need three independent recording groups for train/val/test")
    group_split={group:("train" if index<len(groups)-2 else "val" if index==len(groups)-2 else "test") for index,group in enumerate(groups)}
    frames=[]
    for index,(key,row) in enumerate(sorted(entries.items())):
        source=row["sources"][0] if row["sources"] else None
        splits={group_split[s["group"]] for s in row["sources"]}
        split=next(iter(splits)) if len(splits)==1 else "quarantine"
        name=f"frame_{index:05d}";image_path=Path("images")/split/f"{name}.png"
        (output/image_path).parent.mkdir(parents=True,exist_ok=True)
        Image.frombytes("L",(352,288),row["pixels"]).save(output/image_path)
        frames.append({"name":name,"image":str(image_path),"label":str(Path("labels")/split/f"{name}.txt"),
            "mask":str(Path("masks")/split/f"{name}.png"),"split":split,"pixel_sha256":key,
            "sha256":digest(output/image_path),"screenshots":row["screenshots"],"sources":row["sources"],
            "body_pose":source["body_pose"] if source else None,"label_status":"awaiting_cad_mask" if source else "missing_pose"})
    metadata={"status":"incomplete","names":["h_marker"],"annotation":"visible full H board solid; H paint required",
        "width":352,"height":288,"fov_y_deg":70.88,"camera_position":[-.03,0,0],"groups":group_split,
        "screenshots":len(sources),"unique_frames":len(frames),"recordings":recordings,"frames":frames,
        "collector_sha256":digest(Path(__file__)),"scope":"Simulated CIF pixels and CAD labels; real OV2640 transfer requires separate data."}
    manifest=output/"manifest.json";manifest.write_text(json.dumps(metadata,indent=2)+"\n")
    command=[str(args.blender),"--background","--factory-startup","--python",str(Path(__file__).resolve()),"--","--render-manifest",str(manifest)]
    result=subprocess.run(command,capture_output=True,text=True)
    if result.returncode:raise RuntimeError(result.stdout[-4000:]+result.stderr[-4000:])
    metadata=json.loads(manifest.read_text())
    for frame in metadata["frames"]:
        if frame["split"]=="quarantine" and Path(frame["image"]).parts[1]!="quarantine":
            for kind in ("image","mask","label"):
                old=Path(frame[kind]);new=old.parent.parent/"quarantine"/old.name
                (output/new).parent.mkdir(parents=True,exist_ok=True)
                (output/old).rename(output/new);frame[kind]=str(new)
        if frame["body_pose"] is None:
            frame["mask"]=None;frame["label"]=None
            continue
        if not (output/frame["label"]).is_file():raise ValueError("Missing label")
    # Annotated review sheets are dataset QA artifacts, separate from training pixels.
    for offset in range(0,len(metadata["frames"]),24):
        page=Image.new("RGB",(352*4,312*6),"#202020");draw=ImageDraw.Draw(page)
        for i,frame in enumerate(metadata["frames"][offset:offset+24]):
            x,y=(i%4)*352,(i//4)*312
            page.paste(Image.open(output/frame["image"]).convert("RGB"),(x,y))
            for _,cx,cy,w,h in frame.get("labels",[]):draw.rectangle((x+(cx-w/2)*352,y+(cy-h/2)*288,x+(cx+w/2)*352,y+(cy+h/2)*288),outline="#00ff80",width=2)
            draw.text((x+4,y+291),f"{frame['name']} {frame['split']}",fill="white")
        page.save(output/f"review_{offset//24:03d}.jpg")
    (output/"data.yaml").write_text(f"path: {output}\ntrain: images/train\nval: images/val\ntest: images/test\nnames: [h_marker]\n")
    metadata["status"]="complete";manifest.write_text(json.dumps(metadata,indent=2)+"\n")
    counts={split:sum(frame["split"]==split for frame in metadata["frames"]) for split in ("train","val","test","quarantine")}
    print(json.dumps({"dataset":str(output),"screenshots":len(sources),"unique_frames":len(frames),"splits":counts},indent=2))


def main():
    arguments=sys.argv[sys.argv.index("--")+1:] if "--" in sys.argv else sys.argv[1:]
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--render-manifest",type=Path,help=argparse.SUPPRESS)
    parser.add_argument("--images",type=Path)
    parser.add_argument("--recordings",type=Path,nargs="+")
    parser.add_argument("--output",type=Path)
    parser.add_argument("--sample-period",type=float,default=0.)
    parser.add_argument("--max-samples",type=int,default=32)
    parser.add_argument("--blender",type=Path,default=Path("/Applications/Blender.app/Contents/MacOS/Blender"))
    args=parser.parse_args(arguments)
    if args.render_manifest:render_masks(args.render_manifest)
    else:
        if not args.images or not args.recordings or not args.output:parser.error("--images, --recordings and --output are required")
        if args.sample_period<0 or args.max_samples<0:parser.error("Sampling bounds must be nonnegative")
        collect(args)


if __name__=="__main__":main()
