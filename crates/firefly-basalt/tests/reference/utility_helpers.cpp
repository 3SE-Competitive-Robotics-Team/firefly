// SPDX-License-Identifier: BSD-3-Clause
// Executes original utility headers. GL calls are captured, not rendered.
#include <basalt/utils/vis_utils.h>
#include <basalt/utils/sim_utils.h>
#include <basalt/utils/test_utils.h>
#include <basalt/serialization/eigen_io.h>
#include <cereal/archives/json.hpp>
#include <cereal/types/vector.hpp>
#include <nlohmann/json.hpp>
#include <sstream>
#include <iostream>
using Json=nlohmann::json;
Json drawing;const float* vertices=nullptr;
extern "C" {
void glPushMatrix(){} void glPopMatrix(){}
void glMultMatrixd(const GLdouble* p){Json v=Json::array();for(int r=0;r<4;++r)for(int c=0;c<4;++c)v.push_back(p[c*4+r]);drawing["transform"]=v;}
void glColor3ubv(const GLubyte* p){drawing["color"]={p[0],p[1],p[2]};}
void glLineWidth(GLfloat width){drawing["line_width"]=width;}
void glVertexPointer(GLint,GLenum,GLsizei,const GLvoid* p){vertices=static_cast<const float*>(p);}
void glEnableClientState(GLenum){} void glDisableClientState(GLenum){}
void glDrawArrays(GLenum,GLint,GLsizei count){Json lines=Json::array();for(int i=0;i<count;++i)lines.push_back({vertices[3*i],vertices[3*i+1],vertices[3*i+2]});drawing["lines"]=lines;}
}
int main(){
 Json colors=Json::array(),drawings=Json::array(),checks=Json::array();
 for(float p:{-1.f,0.f,.5f,1.f,1.5f,2.f,2.5f,3.f,3.5f,4.f,4.5f,5.f,5.5f,6.f,7.f}){float r,g,b;getcolor(p,4,r,g,b);colors.push_back({{"p",p},{"rgb",{r,g,b}}});}
 for(float size:{0.f,.125f,1.f,-2.f}){drawing=Json::object();Eigen::Matrix4d transform=Eigen::Matrix4d::Identity();transform(0,3)=.25;render_camera(transform,2.5,cam_color,size);drawing["size"]=size;drawings.push_back(drawing);}
 basalt::SimObservations observations;observations.pos={{.125,-.25},{12.,3.5}};observations.id={1,-2};
 std::ostringstream archive;{cereal::JSONOutputArchive output(archive);output(observations);}
 for(int kind=0;kind<4;++kind){Eigen::Matrix2d analytic=Eigen::Matrix2d::Identity();Eigen::Vector2d initial(.25,-.5);if(kind==1)analytic(0,0)=2;if(kind==2)analytic(0,0)=NAN;std::ostringstream report;auto* previous=std::cerr.rdbuf(report.rdbuf());test_jacobian("probe",analytic,[kind](const Eigen::Vector2d& value)->Eigen::Vector2d{if(kind==3)return Eigen::Vector2d::Constant(NAN);return value;},initial);std::cerr.rdbuf(previous);checks.push_back({{"kind",kind},{"reported",!report.str().empty()}});}
 std::cout<<Json{{"colors",colors},{"drawings",drawings},{"simulation",Json::parse(archive.str())},{"checks",checks}}.dump()<<'\n';
}
