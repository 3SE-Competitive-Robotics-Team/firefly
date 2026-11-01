//! Diagnose raw detection versus refinement using an official image-stage probe.
#[path = "../src/utils/apriltag_subpixel.rs"]
mod subpixel;
use firefly_basalt::thirdparty::ethz_apriltag2::{
    tag_detector::TagDetector, tag_family::TagFamily, tag36h11::CODES,
};
use purecv::core::types::Point2f;
use std::{
    fs::File,
    io::{BufReader, BufWriter},
};
fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<_> = std::env::args().collect();
    assert_eq!(args.len(), 3, "reference-json output-json");
    let input: serde_json::Value = serde_json::from_reader(BufReader::new(File::open(&args[1])?))?;
    let pixels: Vec<u8> = serde_json::from_value(input["pixels"].clone())?;
    let width = input["width"].as_u64().unwrap() as usize;
    let height = input["height"].as_u64().unwrap() as usize;
    let detector = TagDetector::new(TagFamily::new(36, 11, &CODES, 2));
    let raw = detector.extract_tags(width, height, &pixels);
    let mut tags = Vec::new();
    for tag in raw {
        let radius = 2.0_f64.max(0.075 * f64::from(tag.observed_perimeter) / 4. - 1.);
        let window = (radius + 1.).ceil() as usize;
        let mut refined: Vec<_> = tag
            .points
            .iter()
            .map(|&(x, y)| Point2f::new(x, y))
            .collect();
        subpixel::refine(&pixels, width, height, &mut refined, window)?;
        tags.push(serde_json::json!({"id":tag.id,"good":tag.good,"perimeter":tag.observed_perimeter,"radius":radius,"window":window,"initial":tag.points,"refined":refined.iter().map(|p|[p.x,p.y]).collect::<Vec<_>>() }));
    }
    let mut common = Vec::new();
    for tag in input["tags"].as_array().unwrap() {
        let points: Vec<[f32; 2]> = serde_json::from_value(tag["initial"].clone())?;
        let mut refined: Vec<_> = points
            .into_iter()
            .map(|p| Point2f::new(p[0], p[1]))
            .collect();
        subpixel::refine(
            &pixels,
            width,
            height,
            &mut refined,
            tag["window"].as_u64().unwrap() as usize,
        )?;
        common.push(serde_json::json!({"id":tag["id"],"refined":refined.iter().map(|p|[p.x,p.y]).collect::<Vec<_>>() }));
    }
    serde_json::to_writer(
        BufWriter::new(File::create(&args[2])?),
        &serde_json::json!({"tags":tags,"cpp_initial_refined":common}),
    )?;
    Ok(())
}
