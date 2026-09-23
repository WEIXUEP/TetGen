#include "tetgen.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <string>
#include <type_traits>
#include <vector>

#if defined(_WIN32)
#include <fcntl.h>
#include <io.h>
#else
#include <unistd.h>
#endif

namespace {

constexpr std::uint32_t protocol_magic = 0x4f485447U;  // OHTG
constexpr std::uint32_t protocol_version = 4;
constexpr std::uint32_t command_execute = 1;
constexpr std::uint32_t command_shutdown = 2;
constexpr std::uint64_t maximum_items = std::uint64_t{1} << 31;

#if defined(_WIN32)
using IoCount = int;
IoCount read_fd(int fd, void* data, unsigned count) {
  return _read(fd, data, count);
}
IoCount write_fd(int fd, void const* data, unsigned count) {
  return _write(fd, data, count);
}
#else
using IoCount = ssize_t;
IoCount read_fd(int fd, void* data, unsigned count) {
  return ::read(fd, data, count);
}
IoCount write_fd(int fd, void const* data, unsigned count) {
  return ::write(fd, data, count);
}
#endif

bool read_exact(int fd, void* data, std::size_t size) {
  auto* bytes = static_cast<unsigned char*>(data);
  while (size) {
    auto const chunk = static_cast<unsigned>(
        std::min<std::size_t>(size, std::numeric_limits<unsigned>::max()));
    auto const count = read_fd(fd, bytes, chunk);
    if (count <= 0) return false;
    bytes += count;
    size -= static_cast<std::size_t>(count);
  }
  return true;
}

bool write_exact(int fd, void const* data, std::size_t size) {
  auto const* bytes = static_cast<unsigned char const*>(data);
  while (size) {
    auto const chunk = static_cast<unsigned>(
        std::min<std::size_t>(size, std::numeric_limits<unsigned>::max()));
    auto const count = write_fd(fd, bytes, chunk);
    if (count <= 0) return false;
    bytes += count;
    size -= static_cast<std::size_t>(count);
  }
  return true;
}

template <class T>
bool read_scalar(int fd, T* value) {
  static_assert(std::is_trivially_copyable<T>::value, "wire scalar required");
  return read_exact(fd, value, sizeof(T));
}

template <class T>
bool write_scalar(int fd, T value) {
  static_assert(std::is_trivially_copyable<T>::value, "wire scalar required");
  return write_exact(fd, &value, sizeof(T));
}

template <class T>
bool read_array(int fd, T** destination, std::uint64_t count) {
  if (count > maximum_items || count >
          std::numeric_limits<std::size_t>::max() / sizeof(T))
    return false;
  if (!count) {
    *destination = nullptr;
    return true;
  }
  auto* values = new T[static_cast<std::size_t>(count)];
  if (!read_exact(fd, values, static_cast<std::size_t>(count) * sizeof(T))) {
    delete[] values;
    return false;
  }
  *destination = values;
  return true;
}

template <class T>
bool write_array(int fd, T const* values, std::uint64_t count) {
  if (count > maximum_items || (count && !values)) return false;
  return !count || write_exact(
      fd, values, static_cast<std::size_t>(count) * sizeof(T));
}

bool read_string(int fd, std::string* value) {
  std::uint64_t size = 0;
  if (!read_scalar(fd, &size) || size > (std::uint64_t{1} << 20)) return false;
  value->resize(static_cast<std::size_t>(size));
  return !size || read_exact(fd, &(*value)[0], static_cast<std::size_t>(size));
}

bool read_mesh(int fd, tetgenio* mesh) {
  std::int32_t first = 0, points = 0, tets = 0, corners = 0;
  std::int32_t attributes = 0, faces = 0, edges = 0, plc_facets = 0;
  std::int32_t regions = 0;
  std::uint8_t point_markers = 0, point_origins = 0, edge_supports = 0;
  std::uint8_t face_markers = 0, edge_markers = 0, plc_markers = 0;
  if (!read_scalar(fd, &first) || !read_scalar(fd, &points) ||
      !read_scalar(fd, &tets) || !read_scalar(fd, &corners) ||
      !read_scalar(fd, &attributes) || !read_scalar(fd, &faces) ||
      !read_scalar(fd, &edges) || !read_scalar(fd, &plc_facets) ||
      !read_scalar(fd, &regions) || !read_scalar(fd, &point_markers) ||
      !read_scalar(fd, &point_origins) || !read_scalar(fd, &face_markers) ||
      !read_scalar(fd, &edge_markers) || !read_scalar(fd, &edge_supports) ||
      !read_scalar(fd, &plc_markers))
    return false;
  if (points < 0 || tets < 0 || faces < 0 || edges < 0 || corners < 0 ||
      attributes < 0 || plc_facets < 0 || regions < 0)
    return false;
  mesh->firstnumber = first;
  mesh->mesh_dim = 3;
  mesh->numberofpoints = points;
  mesh->numberoftetrahedra = tets;
  mesh->numberofcorners = corners;
  mesh->numberoftetrahedronattributes = attributes;
  mesh->numberoftrifaces = faces;
  mesh->numberofedges = edges;
  REAL* point_origin_values = nullptr;
  int* edge_support_values = nullptr;
  int* plc_triangles = nullptr;
  auto const arrays_ok = read_array(fd, &mesh->pointlist,
          std::uint64_t(points) * 3) &&
      (!point_markers || read_array(fd, &mesh->pointmarkerlist, points)) &&
      (!point_origins || read_array(fd, &point_origin_values, points)) &&
      (!edge_supports || read_array(
          fd, &edge_support_values, std::uint64_t(points) * 2)) &&
      read_array(fd, &mesh->tetrahedronlist,
          std::uint64_t(tets) * static_cast<unsigned>(corners)) &&
      read_array(fd, &mesh->tetrahedronattributelist,
          std::uint64_t(tets) * static_cast<unsigned>(attributes)) &&
      read_array(fd, &mesh->trifacelist, std::uint64_t(faces) * 3) &&
      (!face_markers || read_array(fd, &mesh->trifacemarkerlist, faces)) &&
      read_array(fd, &mesh->edgelist, std::uint64_t(edges) * 2) &&
      (!edge_markers || read_array(fd, &mesh->edgemarkerlist, edges)) &&
      read_array(fd, &plc_triangles, std::uint64_t(plc_facets) * 3) &&
      (!plc_markers || read_array(
          fd, &mesh->facetmarkerlist, plc_facets)) &&
      read_array(fd, &mesh->regionlist, std::uint64_t(regions) * 5);
  if (!arrays_ok) {
    delete[] point_origin_values;
    delete[] edge_support_values;
    delete[] plc_triangles;
    return false;
  }
  mesh->numberofpointattributes = edge_supports ? 3 :
      (point_origins ? 1 : 0);
  if (mesh->numberofpointattributes) {
    mesh->pointattributelist =
        new REAL[std::uint64_t(points) * mesh->numberofpointattributes];
    for (int point = 0; point < points; ++point) {
      auto const offset = point * mesh->numberofpointattributes;
      mesh->pointattributelist[offset] =
          point_origins ? point_origin_values[point] : REAL(-1);
      if (edge_supports) {
        mesh->pointattributelist[offset + 1] =
            edge_support_values[2 * point];
        mesh->pointattributelist[offset + 2] =
            edge_support_values[2 * point + 1];
      }
    }
  }
  delete[] point_origin_values;
  delete[] edge_support_values;
  mesh->numberoffacets = plc_facets;
  mesh->numberofregions = regions;
  if (plc_facets) {
    mesh->facetlist = new tetgenio::facet[plc_facets];
    for (int facet = 0; facet < plc_facets; ++facet) {
      auto& f = mesh->facetlist[facet];
      f.numberofpolygons = 1;
      f.polygonlist = new tetgenio::polygon[1];
      f.numberofholes = 0;
      f.holelist = nullptr;
      f.polygonlist[0].numberofvertices = 3;
      f.polygonlist[0].vertexlist = new int[3];
      for (int corner = 0; corner < 3; ++corner)
        f.polygonlist[0].vertexlist[corner] =
            plc_triangles[3 * facet + corner];
    }
  }
  delete[] plc_triangles;
  return true;
}

bool write_mesh(int fd, tetgenio const& mesh) {
  auto const point_markers = std::uint8_t(mesh.pointmarkerlist != nullptr);
  auto const point_origins = std::uint8_t(
      mesh.pointattributelist != nullptr && mesh.numberofpointattributes > 0);
  auto const edge_supports = std::uint8_t(0);
  auto const face_markers = std::uint8_t(mesh.trifacemarkerlist != nullptr);
  auto const edge_markers = std::uint8_t(mesh.edgemarkerlist != nullptr);
  // PLC facets are input-only. TetGen reports the recovered surface through
  // trifacelist, so output requests carry an empty PLC section.
  auto const plc_markers = std::uint8_t(0);
  return write_scalar(fd, std::int32_t(mesh.firstnumber)) &&
      write_scalar(fd, std::int32_t(mesh.numberofpoints)) &&
      write_scalar(fd, std::int32_t(mesh.numberoftetrahedra)) &&
      write_scalar(fd, std::int32_t(mesh.numberofcorners)) &&
      write_scalar(fd, std::int32_t(mesh.numberoftetrahedronattributes)) &&
      write_scalar(fd, std::int32_t(mesh.numberoftrifaces)) &&
      write_scalar(fd, std::int32_t(mesh.numberofedges)) &&
      write_scalar(fd, std::int32_t(0)) && write_scalar(fd, std::int32_t(0)) &&
      write_scalar(fd, point_markers) && write_scalar(fd, point_origins) &&
      write_scalar(fd, face_markers) && write_scalar(fd, edge_markers) &&
      write_scalar(fd, edge_supports) &&
      write_scalar(fd, plc_markers) &&
      write_array(fd, mesh.pointlist,
          std::uint64_t(mesh.numberofpoints) * 3) &&
      (!point_markers || write_array(
          fd, mesh.pointmarkerlist, mesh.numberofpoints)) &&
      (!point_origins || [&]() {
        std::vector<REAL> origins(mesh.numberofpoints);
        for (int point = 0; point < mesh.numberofpoints; ++point)
          origins[point] = mesh.pointattributelist[
              std::uint64_t(point) * mesh.numberofpointattributes];
        return write_array(fd, origins.data(), origins.size());
      }()) &&
      write_array(fd, mesh.tetrahedronlist,
          std::uint64_t(mesh.numberoftetrahedra) * mesh.numberofcorners) &&
      write_array(fd, mesh.tetrahedronattributelist,
          std::uint64_t(mesh.numberoftetrahedra) *
              mesh.numberoftetrahedronattributes) &&
      write_array(fd, mesh.trifacelist,
          std::uint64_t(mesh.numberoftrifaces) * 3) &&
      (!face_markers || write_array(
          fd, mesh.trifacemarkerlist, mesh.numberoftrifaces)) &&
      write_array(fd, mesh.edgelist, std::uint64_t(mesh.numberofedges) * 2) &&
      (!edge_markers || write_array(
          fd, mesh.edgemarkerlist, mesh.numberofedges));
}

void set_request_environment(
    int budget, double candidate_fraction, double quality_threshold) {
  char budget_text[64], candidate_text[64], threshold_text[64];
  std::snprintf(budget_text, sizeof(budget_text), "%d", budget);
  std::snprintf(candidate_text, sizeof(candidate_text), "%.17g",
      candidate_fraction);
  std::snprintf(threshold_text, sizeof(threshold_text), "%.17g",
      quality_threshold);
#if defined(_WIN32)
  _putenv_s("TETGEN_NEW_POINT_BUDGET", budget < 0 ? "" : budget_text);
  _putenv_s("TETGEN_R2_CANDIDATE_FRACTION", candidate_text);
  _putenv_s("TETGEN_R2_QUALITY_THRESHOLD", threshold_text);
#else
  if (budget < 0) unsetenv("TETGEN_NEW_POINT_BUDGET");
  else setenv("TETGEN_NEW_POINT_BUDGET", budget_text, 1);
  setenv("TETGEN_R2_CANDIDATE_FRACTION", candidate_text, 1);
  setenv("TETGEN_R2_QUALITY_THRESHOLD", threshold_text, 1);
#endif
}

bool execute_request(int input_fd, int output_fd) {
  std::string switches;
  std::int32_t budget = 0;
  double candidate_fraction = 0.0, quality_threshold = 0.0;
  std::int64_t target_tetrahedra = -1;
  std::uint64_t boundary_count = 0;
  std::vector<std::int32_t> boundaries;
  tetgenio input, addin, output;
  if (!read_string(input_fd, &switches) || !read_scalar(input_fd, &budget) ||
      !read_scalar(input_fd, &candidate_fraction) ||
      !read_scalar(input_fd, &quality_threshold) ||
      !read_scalar(input_fd, &target_tetrahedra) ||
      !read_scalar(input_fd, &boundary_count) ||
      boundary_count > maximum_items ||
      boundary_count > std::uint64_t(std::numeric_limits<int>::max()))
    return false;
  boundaries.resize(static_cast<std::size_t>(boundary_count));
  if ((boundary_count && !read_exact(input_fd, boundaries.data(),
          static_cast<std::size_t>(boundary_count) * sizeof(std::int32_t))) ||
      !read_mesh(input_fd, &input) || !read_mesh(input_fd, &addin))
    return false;
  if (target_tetrahedra > std::numeric_limits<long>::max() ||
      (target_tetrahedra >= 0) != !boundaries.empty() ||
      (target_tetrahedra >= 0 &&
          (boundaries.back() != addin.numberofpoints ||
           boundaries.front() <= 0 ||
           !std::is_sorted(boundaries.begin(), boundaries.end()) ||
           std::adjacent_find(boundaries.begin(), boundaries.end()) !=
               boundaries.end()))) return false;
  set_request_environment(budget, candidate_fraction, quality_threshold);
  std::vector<char> mutable_switches(switches.begin(), switches.end());
  mutable_switches.push_back('\0');
  tetgenbehavior behavior;
  std::uint32_t status = 0;
  if (!behavior.parse_commandline(mutable_switches.data())) {
    status = 1;
  } else {
    behavior.new_point_budget = budget;
    behavior.addin_target_tetrahedra = static_cast<long>(target_tetrahedra);
    behavior.addin_candidate_ends.assign(boundaries.begin(), boundaries.end());
    tetrahedralize(&behavior, &input, &output,
        addin.numberofpoints ? &addin : nullptr, nullptr);
  }
  return write_scalar(output_fd, protocol_magic) &&
      write_scalar(output_fd, protocol_version) &&
      write_scalar(output_fd, status) &&
      (status || write_scalar(output_fd,
          std::int32_t(behavior.processed_addin_points))) &&
      (status || write_scalar(output_fd,
          std::int64_t(behavior.addin_stop_tetrahedra))) &&
      (status || write_mesh(output_fd, output));
}

}  // namespace

int main() {
#if defined(_WIN32)
  _setmode(_fileno(stdin), _O_BINARY);
  _setmode(_fileno(stdout), _O_BINARY);
#endif
  int const protocol_output =
#if defined(_WIN32)
      _dup(_fileno(stdout));
#else
      dup(STDOUT_FILENO);
#endif
  if (protocol_output < 0) return 2;
  std::fflush(stdout);
#if defined(_WIN32)
  _dup2(_fileno(stderr), _fileno(stdout));
#else
  dup2(STDERR_FILENO, STDOUT_FILENO);
#endif
  for (;;) {
    std::uint32_t magic = 0, version = 0, command = 0;
    if (!read_scalar(
#if defined(_WIN32)
            _fileno(stdin),
#else
            STDIN_FILENO,
#endif
            &magic) ||
        !read_scalar(
#if defined(_WIN32)
            _fileno(stdin),
#else
            STDIN_FILENO,
#endif
            &version) ||
        !read_scalar(
#if defined(_WIN32)
            _fileno(stdin),
#else
            STDIN_FILENO,
#endif
            &command))
      return 0;
    if (magic != protocol_magic || version != protocol_version) return 3;
    if (command == command_shutdown) return 0;
    if (command != command_execute || !execute_request(
#if defined(_WIN32)
            _fileno(stdin),
#else
            STDIN_FILENO,
#endif
            protocol_output))
      return 4;
  }
}
