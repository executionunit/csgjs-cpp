#define CSGJS_IMPLEMENTATION
#include "csgjs.h"

#include <fstream>

namespace exunit {

bool modeltoply(const char *filename, const csgjs_model &model) {

    std::fstream stream(filename, std::ios_base::out);
    if (stream.is_open()) {
        stream << "ply\n";
        stream << "format ascii 1.0\n";

        // define the vertex elements
        stream << "element vertex " << model.vertices.size() << '\n';
        stream << "property float x\n";
        stream << "property float y\n";
        stream << "property float z\n";
        stream << "property float nx\n";
        stream << "property float ny\n";
        stream << "property float nz\n";

        // define the face elements
        stream << "element face " << model.indices.size() / 3 << '\n';
        stream << "property list uchar int32 vertex_indices\n";
        stream << "end_header\n";

        for (const auto &v : model.vertices) {
            stream << v.pos.x << " " << v.pos.y << " " << v.pos.z << " " << v.normal.x << " " << v.normal.y << " "
                   << v.normal.z << '\n';
        }

        for (int idx = 2; idx < model.indices.size(); idx += 3) {
            stream << "3 " << model.indices[idx - 2] << " " << model.indices[idx - 1] << " " << model.indices[idx - 0]
                   << '\n';
        }
        return true;
    }

    return false;
}

} // namespace exunit

int main(int argc, char **arvc) {

    auto cube1 = csgsmodel_cube();
    auto cube2 = csgsmodel_cube({1, 0, 0}, {0.8f, 0.8f, 0.8f});

    auto model = csgjs_subtract(cube1, cube2);

    exunit::modeltoply("temp.ply", model);

    return 0;
}