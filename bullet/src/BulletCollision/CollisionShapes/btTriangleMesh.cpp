/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2009 Erwin Coumans  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/


#include "btTriangleMesh.h"



btTriangleMesh::btTriangleMesh (bool use32bitIndices,bool use4componentVertices)
:m_use32bitIndices(use32bitIndices),
m_use4componentVertices(use4componentVertices),
m_weldingThreshold(0.0)
{
	btIndexedMesh meshIndex;
	meshIndex.m_numTriangles = 0;
	meshIndex.m_numVertices = 0;
	meshIndex.m_indexType = PHY_INTEGER;
	meshIndex.m_triangleIndexBase = 0;
	meshIndex.m_triangleIndexStride = 3*sizeof(int);
	meshIndex.m_vertexBase = 0;
	meshIndex.m_vertexStride = sizeof(btVector3);
	m_indexedMeshes.push_back(meshIndex);

	if (m_use32bitIndices)
	{
		m_indexedMeshes[0].m_numTriangles = m_32bitIndices.size()/3;
		m_indexedMeshes[0].m_triangleIndexBase = 0;
		m_indexedMeshes[0].m_indexType = PHY_INTEGER;
		m_indexedMeshes[0].m_triangleIndexStride = 3*sizeof(int);
	} else
	{
		m_indexedMeshes[0].m_numTriangles = m_16bitIndices.size()/3;
		m_indexedMeshes[0].m_triangleIndexBase = 0;
		m_indexedMeshes[0].m_indexType = PHY_SHORT;
		m_indexedMeshes[0].m_triangleIndexStride = 3*sizeof(short int);
	}

	if (m_use4componentVertices)
	{
		m_indexedMeshes[0].m_numVertices = m_4componentVertices.size();
		m_indexedMeshes[0].m_vertexBase = 0;
		m_indexedMeshes[0].m_vertexStride = sizeof(btVector3);
	} else
	{
		m_indexedMeshes[0].m_numVertices = m_3componentVertices.size()/3;
		m_indexedMeshes[0].m_vertexBase = 0;
		m_indexedMeshes[0].m_vertexStride = 3*sizeof(btScalar);
	}

	m_hasVNorms = false;
}

void	btTriangleMesh::addIndex(int index)
{
	if (m_use32bitIndices)
	{
		m_32bitIndices.push_back(index);
		m_indexedMeshes[0].m_triangleIndexBase = (unsigned char*) &m_32bitIndices[0];
	} else
	{
		m_16bitIndices.push_back(index);
		m_indexedMeshes[0].m_triangleIndexBase = (unsigned char*) &m_16bitIndices[0];
	}
}


int	btTriangleMesh::findOrAddVertex(const btVector3& vertex, bool removeDuplicateVertices)
{
	//return index of new/existing vertex
	///@todo: could use acceleration structure for this
	if (m_use4componentVertices)
	{
		if (removeDuplicateVertices)
			{
			for (int i=0;i< m_4componentVertices.size();i++)
			{
				if ((m_4componentVertices[i]-vertex).length2() <= m_weldingThreshold)
				{
					return i;
				}
			}
		}
		m_indexedMeshes[0].m_numVertices++;
		m_4componentVertices.push_back(vertex);
		m_indexedMeshes[0].m_vertexBase = (unsigned char*)&m_4componentVertices[0];

		return m_4componentVertices.size()-1;
		
	} else
	{
		
		if (removeDuplicateVertices)
		{
			for (int i=0;i< m_3componentVertices.size();i+=3)
			{
				btVector3 vtx(m_3componentVertices[i],m_3componentVertices[i+1],m_3componentVertices[i+2]);
				if ((vtx-vertex).length2() <= m_weldingThreshold)
				{
					return i/3;
				}
			}
		}
		m_3componentVertices.push_back(vertex.getX());
		m_3componentVertices.push_back(vertex.getY());
		m_3componentVertices.push_back(vertex.getZ());
		m_indexedMeshes[0].m_numVertices++;
		m_indexedMeshes[0].m_vertexBase = (unsigned char*)&m_3componentVertices[0];
		return (m_3componentVertices.size()/3)-1;
	}

}
		
void	btTriangleMesh::addTriangle(const btVector3& vertex0,const btVector3& vertex1,const btVector3& vertex2,bool removeDuplicateVertices)
{
	m_indexedMeshes[0].m_numTriangles++;
	addIndex(findOrAddVertex(vertex0,removeDuplicateVertices));
	addIndex(findOrAddVertex(vertex1,removeDuplicateVertices));
	addIndex(findOrAddVertex(vertex2,removeDuplicateVertices));

}

float btTriangleMesh::getP1P2P3(unsigned int indx) const
{
	return m_p1p2p3[indx];
}



btVector3 btTriangleMesh::getInterpolatedNormal2(unsigned int index, const btVector3& position) const
{
	btVector3 p1, p2, p3;
	btVector3 n1, n2, n3;
	/*
	if (m_can_be_transformed)
	{
		// If the object of this mesh can be transformed, we need to compute
		// the updated positions and normals before interpolating.
		btVector3 q1, q2, q3;
		getTriangle(index, &q1, &q2, &q3);
		const btTransform& tf = m_body->getWorldTransform();
		// The triangle verteces must be moved according to the transform of the body
		p1 = tf(q1);
		p2 = tf(q2);
		p3 = tf(q3);
		// The normals must be rotated according to the transform of the body
		btVector3 m1, m2, m3;
		getNormals(index, &m1, &m2, &m3);
		n1 = tf.getBasis() * m1;
		n2 = tf.getBasis() * m2;
		n3 = tf.getBasis() * m3;
	}
	else
	*/
	{

		int i = m_32bitIndices[index * 3 + 0];
		int j = m_32bitIndices[index * 3 + 1];
		int k = m_32bitIndices[index * 3 + 2];

		p1 = m_4componentVertices[i];
		p2 = m_4componentVertices[j];
		p3 = m_4componentVertices[k];

		n1 = m_3componentNormals[i];
		n2 = m_3componentNormals[j];
		n3 = m_3componentNormals[k];

		//getNormals(index, &n1, &n2, &n3);
	}

	// Compute the Barycentric coordinates of position inside  triangle
	// p1, p2, p3.

	float p1p2p3 = getP1P2P3(index);

	// Area of BCP
	btScalar p2p3p = (p3 - p2).cross(position - p2).length2();

	// Area of CAP
	btVector3 edge2 = p3 - p1;
	btScalar p3p1p = edge2.cross(position - p3).length2();
	btScalar s = btSqrt(p2p3p / p1p2p3);
	btScalar t = btSqrt(p3p1p / p1p2p3);
	btScalar w = 1.0f - s - t;

#ifdef NORMAL_DEBUGGING
	btVector3 regen_position = s * p1 + t * p2 + w * p3;

	if ((regen_position - position).length2() >= 0.0001f)
	{
		printf("bary:\n");
		printf("new: %f %f %f\n", regen_position.getX(), regen_position.getY(), regen_position.getZ());
		printf("old: %f %f %f\n", position.getX(), position.getY(), position.getZ());
		printf("stw: %f %f %f\n", s, t, w);
		printf("p1:  %f %f %f\n", p1.getX(), p1.getY(), p1.getZ());
		printf("p2:  %f %f %f\n", p2.getX(), p2.getY(), p2.getZ());
		printf("p3:  %f %f %f\n", p3.getX(), p3.getY(), p3.getZ());
		printf("pos: %f %f %f\n", position.getX(), position.getY(), position.getZ());
	}
#endif

	return s * n1 + t * n2 + w * n3;
}   // getInterpolatedNormal

int btTriangleMesh::getNumTriangles() const
{
	if (m_use32bitIndices)
	{
		return m_32bitIndices.size() / 3;
	}
	return m_16bitIndices.size() / 3;
}

void btTriangleMesh::preallocateVertices(int numverts)
{
	if (m_use4componentVertices)
	{
		m_4componentVertices.reserve(numverts);
	} else
	{
		m_3componentVertices.reserve(numverts);
	}
}

btVector3 btTriangleMesh::getVertexNormal(int index)
{
	return m_3componentNormals[index];
}

bool btTriangleMesh::hasVertexNormals()
{
	return m_hasVNorms;
}

void btTriangleMesh::computeVertexNormals()
{
	m_3componentNormals.reserve(m_4componentVertices.size());

	for (int n = 0; n < m_4componentVertices.size(); n++)
	{
		m_3componentNormals[n].setValue(0, 0, 0);
	}

	for (int n = 0; n < m_32bitIndices.size(); n += 3)
	{
		int i = m_32bitIndices[n + 0];
		int j = m_32bitIndices[n + 1];
		int k = m_32bitIndices[n + 2];

		btVector3 wv0 = m_4componentVertices[i];
		btVector3 wv1 = m_4componentVertices[j];
		btVector3 wv2 = m_4componentVertices[k];

		btVector3 normal = (wv1 - wv0).cross(wv2 - wv0);
		normal.normalize();

#ifdef NORMAL_DEBUGGING
		printf("tri %d %d %d \n", i, j, k);
		printf("v1 %f %f %f \n", wv0.getX(), wv0.getY(), wv0.getZ());
		printf("v2 %f %f %f \n", wv1.getX(), wv1.getY(), wv1.getZ());
		printf("v3 %f %f %f \n", wv2.getX(), wv2.getY(), wv2.getZ());
		printf("n %f %f %f \n", normal.getX(), normal.getY(), normal.getZ());
#endif

		m_3componentNormals[i] += normal;
		m_3componentNormals[i].normalize();

		m_3componentNormals[j] += normal;
		m_3componentNormals[j].normalize();

		m_3componentNormals[k] += normal;
		m_3componentNormals[k].normalize();


		btVector3 edge1 = wv1 - wv0;
		btVector3 edge2 = wv2 - wv0;
		m_p1p2p3.push_back(edge1.cross(edge2).length2());
	}


	/*
	const unsigned char* vertexbase;
	int num_verts;
	PHY_ScalarType type;
	int stride;

	const unsigned char* indexbase;
	int indexstride;
	int numfaces;
	PHY_ScalarType indicestype;
	int numverts = 0;
	vnorms = new btVector3*[this->getNumSubParts()];

	for (int partId = 0; partId < this->getNumSubParts(); partId++)
	{
		this->getLockedReadOnlyVertexIndexBase(&vertexbase, numverts, type, stride, &indexbase, indexstride, numfaces, indicestype, partId);
		vnorms[partId] = new btVector3[numverts];

		for (int triangle = 0; triangle < numfaces; triangle++)
		{
			const unsigned int* indices = (const unsigned int*)(indexbase + triangle * indexstride);

			unsigned int i = indices[0], j = indices[1], k = indices[2];

			float* gp1 = (float*)(vertexbase + i * stride);
			float* gp2 = (float*)(vertexbase + j * stride);
			float* gp3 = (float*)(vertexbase + k * stride);

			btVector3 wv0, wv1, wv2;
			wv0 = btVector3(gp1[0], gp1[1], gp1[2]);
			wv1 = btVector3(gp2[0], gp2[1], gp2[2]);
			wv2 = btVector3(gp3[0], gp3[1], gp3[2]);

			btVector3 normal = (wv1 - wv0).cross(wv2 - wv0);
			normal.normalize();

			vnorms[partId][i] += normal;
			vnorms[partId][i].normalize();

			vnorms[partId][j] += normal;
			vnorms[partId][j].normalize();

			vnorms[partId][k] += normal;
			vnorms[partId][k].normalize();
		}

		this->unLockReadOnlyVertexBase(partId);
	}
	*/

	m_hasVNorms = true;

}

btVector3 btTriangleMesh::interpolateMeshNormal(const btTransform& transform, btStridingMeshInterface* mesh_interface, int subpart, int triangle, const btVector3& position)
{
	const unsigned char* vertexbase;
	int num_verts;
	PHY_ScalarType type;
	int stride;

	const unsigned char* indexbase;
	int indexstride;
	int numfaces;
	PHY_ScalarType indicestype;

	int numverts = 0;
	mesh_interface->getLockedReadOnlyVertexIndexBase(&vertexbase, numverts, type, stride, &indexbase, indexstride, numfaces, indicestype, subpart);

	// Calculate new barycentric coordinates
	const unsigned int* indices = (const unsigned int*)(indexbase + triangle * indexstride);
	unsigned int i = indices[0], j = indices[1], k = indices[2];
	//StrideVertexAccessor positions(vertexbase, stride, 0);

	const btVector3& meshScaling = mesh_interface->getScaling();
	btVector3 barry;

	if (type == PHY_FLOAT)
	{
		float* gp1= (float*)(vertexbase + i * stride);
		float* gp2 = (float*)(vertexbase + j * stride);
		float* gp3 = (float*)(vertexbase + k * stride);
		
		btVector3 p1 = btVector3(gp1[0] * meshScaling.getX(), gp1[1] * meshScaling.getY(), gp1[2] * meshScaling.getZ());
		btVector3 p2 = btVector3(gp2[0] * meshScaling.getX(), gp2[1] * meshScaling.getY(), gp2[2] * meshScaling.getZ());
		btVector3 p3 = btVector3(gp3[0] * meshScaling.getX(), gp3[1] * meshScaling.getY(), gp3[2] * meshScaling.getZ());

		barry = this->barycentricCoordinates(transform.invXform(position), p1, p2, p3);
	}
	else
	{
		double* gp1 = (double*)(vertexbase + i * stride);
		double* gp2 = (double*)(vertexbase + j * stride);
		double* gp3 = (double*)(vertexbase + k * stride);

		btVector3 p1 = btVector3(gp1[0] * meshScaling.getX(), gp1[1] * meshScaling.getY(), gp1[2] * meshScaling.getZ());
		btVector3 p2 = btVector3(gp2[0] * meshScaling.getX(), gp2[1] * meshScaling.getY(), gp2[2] * meshScaling.getZ());
		btVector3 p3 = btVector3(gp3[0] * meshScaling.getX(), gp3[1] * meshScaling.getY(), gp3[2] * meshScaling.getZ());

		barry = this->barycentricCoordinates(transform.invXform(position), p1, p2, p3);
	}

	
	// ..
	// btVector3 n1 = this->getVertexNormal(i);
	// btVector3 n2 = this->getVertexNormal(j);
	// btVector3 n3 = this->getVertexNormal(k);
	// printf("SmoothVehicleRaycaster: Trace Mesh Vertex Normals I: (%f x %f x %f) -> J: (%f x %f x %f) -> K: (%f x %f x %f)\n", n1.x(), n1.y(), n1.z(), n2.x(), n2.y(), n2.z(), n3.x(), n3.y(), n3.z());
	// ..
	// Interpolate from barycentric coordinates
	// ..
	btVector3 result = barry.x() * this->getVertexNormal(i) + barry.y() * this->getVertexNormal(j) + barry.z() * this->getVertexNormal(k);
	// ..
	// Transform back into world space
	// ..
	result = transform.getBasis() * result;
	result.normalize();
	mesh_interface->unLockReadOnlyVertexBase(subpart);
	return result;
}

btVector3 btTriangleMesh::barycentricCoordinates(const btVector3& position, const btVector3& p1, const btVector3& p2, const btVector3& p3)
{
	btVector3 edge1 = p2 - p1;
	btVector3 edge2 = p3 - p1;
	// Area of triangle ABC
	btScalar p1p2p3 = edge1.cross(edge2).length2();
	// Area of BCP
	btScalar p2p3p = (p3 - p2).cross(position - p2).length2(); // Area of CAP
	btScalar p3p1p = edge2.cross(position - p3).length2();
	btScalar s = btSqrt(p2p3p / p1p2p3);
	btScalar t = btSqrt(p3p1p / p1p2p3);
	btScalar w = 1.0f - s - t;
	return btVector3(s, t, w);
}


void btTriangleMesh::preallocateIndices(int numindices)
{
	if (m_use32bitIndices)
	{
		m_32bitIndices.reserve(numindices);
	} else
	{
		m_16bitIndices.reserve(numindices);
	}
}
