#include "ITMMesh.h"

#include <fstream>

namespace ITMLib
{
namespace Objects
{

////////////////////////////////////////////////////////////////////////////////
void ITMMesh::WriteOBJ(const char *fileName)
{
    ORUtils::MemoryBlock<Triangle> *cpu_triangles; bool shoulDelete = false;
    if (memoryType == MEMORYDEVICE_CUDA)
    {
        cpu_triangles = new ORUtils::MemoryBlock<Triangle>(noMaxTriangles, MEMORYDEVICE_CPU);
        cpu_triangles->SetFrom(triangles, ORUtils::MemoryBlock<Triangle>::CUDA_TO_CPU);
        shoulDelete = true;
    }
    else cpu_triangles = triangles;

    Triangle *triangleArray = cpu_triangles->GetData(MEMORYDEVICE_CPU);

    FILE *f = fopen(fileName, "w+");
    if (f != NULL)
    {
        for (uint i = 0; i < noTotalTriangles; i++)
        {
            fprintf(f, "v %f %f %f\n", triangleArray[i].p0.x, triangleArray[i].p0.y, triangleArray[i].p0.z);
            fprintf(f, "v %f %f %f\n", triangleArray[i].p1.x, triangleArray[i].p1.y, triangleArray[i].p1.z);
            fprintf(f, "v %f %f %f\n", triangleArray[i].p2.x, triangleArray[i].p2.y, triangleArray[i].p2.z);
        }

        for (uint i = 0; i<noTotalTriangles; i++) fprintf(f, "f %d %d %d\n", i * 3 + 2 + 1, i * 3 + 1 + 1, i * 3 + 0 + 1);
        fclose(f);
    }

    if (shoulDelete) delete cpu_triangles;
}

////////////////////////////////////////////////////////////////////////////////
void ITMMesh::WriteSTL(const char *fileName)
{
    ORUtils::MemoryBlock<Triangle> *cpu_triangles; bool shoulDelete = false;
    if (memoryType == MEMORYDEVICE_CUDA)
    {
        cpu_triangles = new ORUtils::MemoryBlock<Triangle>(noMaxTriangles, MEMORYDEVICE_CPU);
        cpu_triangles->SetFrom(triangles, ORUtils::MemoryBlock<Triangle>::CUDA_TO_CPU);
        shoulDelete = true;
    }
    else cpu_triangles = triangles;

    Triangle *triangleArray = cpu_triangles->GetData(MEMORYDEVICE_CPU);

    FILE *f = fopen(fileName, "wb+");

    if (f != NULL) {
        for (int i = 0; i < 80; i++) fwrite(" ", sizeof(char), 1, f);

        fwrite(&noTotalTriangles, sizeof(int), 1, f);

        float zero = 0.0f; short attribute = 0;
        for (uint i = 0; i < noTotalTriangles; i++)
        {
            fwrite(&zero, sizeof(float), 1, f); fwrite(&zero, sizeof(float), 1, f); fwrite(&zero, sizeof(float), 1, f);

            fwrite(&triangleArray[i].p2.x, sizeof(float), 1, f);
            fwrite(&triangleArray[i].p2.y, sizeof(float), 1, f);
            fwrite(&triangleArray[i].p2.z, sizeof(float), 1, f);

            fwrite(&triangleArray[i].p1.x, sizeof(float), 1, f);
            fwrite(&triangleArray[i].p1.y, sizeof(float), 1, f);
            fwrite(&triangleArray[i].p1.z, sizeof(float), 1, f);

            fwrite(&triangleArray[i].p0.x, sizeof(float), 1, f);
            fwrite(&triangleArray[i].p0.y, sizeof(float), 1, f);
            fwrite(&triangleArray[i].p0.z, sizeof(float), 1, f);

            fwrite(&attribute, sizeof(short), 1, f);

            //fprintf(f, "v %f %f %f\n", triangleArray[i].p0.x, triangleArray[i].p0.y, triangleArray[i].p0.z);
            //fprintf(f, "v %f %f %f\n", triangleArray[i].p1.x, triangleArray[i].p1.y, triangleArray[i].p1.z);
            //fprintf(f, "v %f %f %f\n", triangleArray[i].p2.x, triangleArray[i].p2.y, triangleArray[i].p2.z);
        }

        //for (uint i = 0; i<noTotalTriangles; i++) fprintf(f, "f %d %d %d\n", i * 3 + 2 + 1, i * 3 + 1 + 1, i * 3 + 0 + 1);
        fclose(f);
    }

    if (shoulDelete) delete cpu_triangles;
}

////////////////////////////////////////////////////////////////////////////////
void ITMMesh::WritePCD(const char *fileName)
{
    // Ensure to have CPU memory (convert from CUDA if needed).
    ORUtils::MemoryBlock<Triangle> *cpu_triangles;
    bool shouldDelete = false;
    if (memoryType == MEMORYDEVICE_CUDA)
    {
        cpu_triangles = new ORUtils::MemoryBlock<Triangle>(noMaxTriangles, MEMORYDEVICE_CPU);
        cpu_triangles->SetFrom(triangles, ORUtils::MemoryBlock<Triangle>::CUDA_TO_CPU);
        shouldDelete = true;
    }
    else cpu_triangles = triangles;

    Triangle *triangleArray = cpu_triangles->GetData(MEMORYDEVICE_CPU);

    // Write PCD file header.
    // NOTE: this is a text section and existing files are overwritten.
    std::ofstream fStream;
    fStream.open(fileName, std::ofstream::out | std::ofstream::trunc);

    fStream << "# .PCD v.7 - Point Cloud Data file format" << std::endl;
    fStream << "VERSION .7" << std::endl;
    fStream << "FIELDS x y z" << std::endl;
    fStream << "SIZE 4 4 4" << std::endl;
    fStream << "TYPE F F F" << std::endl;
    fStream << "COUNT 1 1 1" << std::endl;
    fStream << "WIDTH " << 3 * noTotalTriangles << std::endl;
    fStream << "HEIGHT 1" << std::endl;
    fStream << "VIEWPOINT 0 0 0 1 0 0 0" << std::endl;
    fStream << "POINTS " << 3 * noTotalTriangles << std::endl;
    fStream << "DATA binary" << std::endl;

    fStream.close();

    // Write the points cloud.
    // NOTE: the vertices data is appended to the header.
    fStream.open(fileName, std::ofstream::out | std::ofstream::app | std::ofstream::binary );

//    float bob = 2.45f;
//    fStream.write((char*)&bob, sizeof(bob));

    for (uint i = 0; i < noTotalTriangles; i++)
    {
        // Vertices order is 2, 1, 0.
        fStream.write((char*)&triangleArray[i].p2.x, sizeof(float));
        fStream.write((char*)&triangleArray[i].p2.y, sizeof(float));
        fStream.write((char*)&triangleArray[i].p2.z, sizeof(float));

        fStream.write((char*)&triangleArray[i].p1.x, sizeof(float));
        fStream.write((char*)&triangleArray[i].p1.y, sizeof(float));
        fStream.write((char*)&triangleArray[i].p1.z, sizeof(float));

        fStream.write((char*)&triangleArray[i].p0.x, sizeof(float));
        fStream.write((char*)&triangleArray[i].p0.y, sizeof(float));
        fStream.write((char*)&triangleArray[i].p0.z, sizeof(float));
    }

    if (shouldDelete)
        delete cpu_triangles;

    fStream.close();
}

} // namespace Objects.
} // namespace ITMLib.
