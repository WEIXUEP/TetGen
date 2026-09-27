#include "tetgen.h"

#include <cstdio>

namespace {
bool check(bool condition, const char* message) {
  if (!condition) std::fprintf(stderr, "%s\n", message);
  return condition;
}
}

int main() {
  tetgenbehavior behavior;
  tetgenio input;
  tetgenmesh mesh;
  mesh.b = &behavior;
  mesh.in = &input;
  behavior.plc = 1;
  bool ok = true;
  mesh.st_segref_count = 5;
  ok &= check(!mesh.should_smooth_vertices(), "segment-only PLC must not start smoothing");
  mesh.st_volref_count = 1;
  ok &= check(mesh.should_smooth_vertices(), "PLC volume points must start smoothing");
  mesh.st_volref_count = 0;
  mesh.st_facref_count = 1;
  ok &= check(mesh.should_smooth_vertices(), "PLC facet points must start smoothing");
  behavior.smooth_maxiter = 0;
  ok &= check(!mesh.should_smooth_vertices(), "disabled smoothing must stay disabled");
  behavior.smooth_maxiter = 7;
  behavior.plc = behavior.quality = 0;
  ok &= check(!mesh.should_smooth_vertices(), "unconstrained meshing must not start smoothing");
  behavior.quality = 1;
  ok &= check(mesh.should_smooth_vertices(), "quality refinement retains historical trigger");

  // An input slot and a new Steiner slot exercise the same queue classifier.
  REAL vertex[16] = {};
  mesh.pointmarkindex = 3;
  input.firstnumber = 1;
  input.numberofpoints = 1;
  input.pointmarkerlist = new int[1];
  mesh.setpointmark(vertex, 1);
  const tetgenmesh::verttype types[] = {
      tetgenmesh::FREEVOLVERTEX, tetgenmesh::FREEFACETVERTEX,
      tetgenmesh::FREESEGVERTEX};
  for (auto type : types) {
    mesh.setpointtype(vertex, type);
    input.pointmarkerlist[0] = 0;
    ok &= check(mesh.smoothing_vertex_type(vertex) == type,
        "unmarked PLC inputs must retain their native FREE type");
    input.pointmarkerlist[0] = -2147482999;
    ok &= check(mesh.smoothing_vertex_type(vertex) == tetgenmesh::UNUSEDVERTEX,
        "CAD-fixed inputs must not enter any smoothing queue");
    mesh.setpointmark(vertex, 2);
    ok &= check(mesh.smoothing_vertex_type(vertex) == type,
        "new Steiner points must not inherit an unrelated input lock");
    mesh.setpointmark(vertex, 1);
  }
  input.pointmarkerlist[0] = 0;
  mesh.setpointtype(vertex, tetgenmesh::RIDGEVERTEX);
  ok &= check(mesh.smoothing_vertex_type(vertex) == tetgenmesh::UNUSEDVERTEX,
      "native ridge points must remain fixed");
  mesh.setpointtype(vertex, tetgenmesh::VOLVERTEX);
  ok &= check(mesh.smoothing_vertex_type(vertex) == tetgenmesh::UNUSEDVERTEX,
      "ordinary input types must not be promoted to FREE vertices");
  return ok ? 0 : 1;
}
