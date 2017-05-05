// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Utils/ITMLibDefines.h"
#include "../../ORUtils/Image.h"

#include <stdlib.h>

namespace ITMLib
{
	namespace Objects
	{
		class ITMMesh
		{
		public:
			struct Triangle { Vector3f p0, p1, p2; };
		
			MemoryDeviceType memoryType;

			uint noTotalTriangles;
			static const uint noMaxTriangles = SDF_LOCAL_BLOCK_NUM * 32;

			ORUtils::MemoryBlock<Triangle> *triangles;

			explicit ITMMesh(MemoryDeviceType memoryType)
			{
				this->memoryType = memoryType;
				this->noTotalTriangles = 0;

				triangles = new ORUtils::MemoryBlock<Triangle>(noMaxTriangles, memoryType);
			}

            void WriteOBJ(const char *fileName);
            void WriteSTL(const char *fileName);
            void WritePCD(const char *fileName);

            ~ITMMesh()
			{
				delete triangles;
			}

			// Suppress the default copy constructor and assignment operator
			ITMMesh(const ITMMesh&);
			ITMMesh& operator=(const ITMMesh&);
		};
	}
}
