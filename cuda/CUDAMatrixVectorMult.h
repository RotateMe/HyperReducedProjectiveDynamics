#pragma once
#define NOMINMAX
#include <Windows.h>

// Uncomment the #define below to enable direct mapping of the vertex positions to the
// GPU. This requires the glew libraries to be linked.
// Then use PD::ProjDynSimulator::initializeGPUVPosMapping(GLuint bufferId) on a simulator
// with the Id of the buffer containing the vertex positions.
// Note however, that in this case, the vertices passed to the simulator must be in the
// same ordering as in the buffer.

// #define ENABLE_DIRECT_BUFFER_MAP
#ifdef ENABLE_DIRECT_BUFFER_MAP
#ifndef __gl_h_
#ifndef __GL_H_
#include <GL\glew.h>
#endif
#endif
#endif
#include <cuda_runtime.h>
#include <iostream>
#include "cuda_gl_interop.h"
#include "cublas_v2.h"
#include "cusparse_v2.h"
#include "ProjDynTypeDef.h"
#include "StopWatch.h"


class CUDAMatrixVectorMultiplier {
public:
	CUDAMatrixVectorMultiplier(PD::PDMatrix& mat);
	CUDAMatrixVectorMultiplier(PD::PDMatrix& mat, PD::PDVector& masses);
	~CUDAMatrixVectorMultiplier();
	void mult(const void* inData, void* outData, PD::PDScalar& alpha, bool tranpose = false, int coord = 0, int cutoff = -1);
	void printTimings();

#ifdef ENABLE_DIRECT_BUFFER_MAP
	void setGLBuffer(GLUint);
#endif

private:
	static bool cublasLibInitialized;
	static cublasHandle_t cublasLibHandle;
	static PD::PDScalar cublasZero;

	/* m_M = number of cols, m_N = number of rows */
	unsigned int m_M, m_N;

#ifdef ENABLE_DIRECT_BUFFER_MAP
	GLuint m_glbufferId;
	float* m_glArrayPtr;
#endif

	int m_massesSize;

	PD::PDScalar* m_cudaMat;
	PD::PDScalar* m_cudaInVec;
	PD::PDScalar* m_cudaOutVec;
	PD::PDScalar* m_cudaMassesVec;

	StopWatch m_multTime;
	StopWatch m_getTime;
	StopWatch m_setTime;
};

class CUSparseMatrixVectorMultiplier {
public:
	CUSparseMatrixVectorMultiplier(PD::PDSparseMatrix& mat);
	void mult(const void* inData, void* outData, PD::PDScalar& alpha);
private:
	static cusparseHandle_t cusparseLibHandle;
	static bool cusparseLibInitialized;
	int m_nnz;
	PD::PDScalar* m_cudaInVec;
	PD::PDScalar* m_cudaOutVec;
	PD::PDScalar* m_cudaMatData;
	cusparseMatDescr_t m_desc;
	double m_alpha;
	double m_zero;
	int* m_cudaColPtr;
	int* m_cudaRowInd;
	/* m_M = number of cols, m_N = number of rows */
	unsigned int m_M, m_N;
};