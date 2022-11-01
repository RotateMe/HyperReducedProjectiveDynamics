#include "CUDAMatrixVectorMult.h"
#include "doubleToFloatDeviceCpy.cuh"

bool CUDAMatrixVectorMultiplier::cublasLibInitialized = false;
PD::PDScalar CUDAMatrixVectorMultiplier::cublasZero = 0.;
cublasHandle_t CUDAMatrixVectorMultiplier::cublasLibHandle = 0;

bool CUSparseMatrixVectorMultiplier::cusparseLibInitialized = false;
cusparseHandle_t CUSparseMatrixVectorMultiplier::cusparseLibHandle = 0;

CUDAMatrixVectorMultiplier::CUDAMatrixVectorMultiplier(PD::PDMatrix & mat)
	:
	m_multTime(10000, 10000),
	m_getTime(10000, 10000),
	m_setTime(10000, 10000),
	m_cudaInVec(nullptr),
	m_cudaOutVec(nullptr),
	m_cudaMassesVec(nullptr),
	m_cudaMat(nullptr),
	m_massesSize(0)
{
	if (!cublasLibInitialized) {
		cublasCreate(&cublasLibHandle);
		cublasLibInitialized = true;
	}

	m_N = mat.rows();
	m_M = mat.cols();

	if (cudaMalloc((void**)&m_cudaInVec, sizeof(PD::PDScalar)*m_M) != cudaError::cudaSuccess) {
		std::cout << "Fatal error: Could not allocate memory for in-vector in CUDA" << std::endl;
		return;
	}
	if (cudaMalloc((void**)&m_cudaMat, sizeof(PD::PDScalar)*m_N*m_M) != cudaError::cudaSuccess) {
		std::cout << "Fatal error: Could not allocate memory for matrix in CUDA" << std::endl;
		return;
	}

	if (cudaMalloc((void**)&m_cudaOutVec, sizeof(PD::PDScalar)*m_N) != cudaError::cudaSuccess) {
		std::cout << "Fatal error: Could not allocate memory for out-vector in CUDA" << std::endl;
		return;
	}

	// If the matrix has more columns then rows, we need to upload it transposed, since
	// cublasDgemv expects a matrix with more rows than columns...
	if (m_N < m_M) {
		PD::PDMatrix matT = mat.transpose();
		const void* matDataPointer = (const void*)matT.data();
		if (cublasSetMatrix(m_M, m_N, sizeof(PD::PDScalar), matDataPointer, m_M, (void*)m_cudaMat, m_M) != cublasStatus_t::CUBLAS_STATUS_SUCCESS) {
			std::cout << "Fatal error: Could not set data for the matrix with CUBLAS." << std::endl;
		}
	}
	else {
		const void* matDataPointer = (const void*)mat.data();
		if (cublasSetMatrix(m_N, m_M, sizeof(PD::PDScalar), matDataPointer, m_N, (void*)m_cudaMat, m_N) != cublasStatus_t::CUBLAS_STATUS_SUCCESS) {
			std::cout << "Fatal error: Could not set data for the matrix with CUBLAS." << std::endl;
		}
	}


}

CUDAMatrixVectorMultiplier::CUDAMatrixVectorMultiplier(PD::PDMatrix & mat, PD::PDVector& masses) 
	:
	CUDAMatrixVectorMultiplier(mat)
{
	m_massesSize = masses.rows();
	if (cudaMalloc((void**)&m_cudaMassesVec, sizeof(PD::PDScalar)*m_massesSize) != cudaError::cudaSuccess) {
		std::cout << "Fatal error: Could not allocate memory for masses vector in CUDA" << std::endl;
		return;
	}
	if (cublasSetVector(m_massesSize, sizeof(PD::PDScalar), masses.data(), 1, (void*)(m_cudaMassesVec), 1) != cublasStatus_t::CUBLAS_STATUS_SUCCESS) {
		std::cout << "Fatal error: Could not set data for the masses vector with CUBLAS." << std::endl;
		return;
	}
	cudaDeviceSynchronize();
}

CUDAMatrixVectorMultiplier::~CUDAMatrixVectorMultiplier()
{
	cudaFree(m_cudaMat);
	cudaFree(m_cudaInVec);
	cudaFree(m_cudaOutVec);
}



void CUDAMatrixVectorMultiplier::mult(const void* inData, void* outData, PD::PDScalar& alpha, bool transpose, int coord, int cutoff)
{
	m_setTime.startStopWatch();
	if (!transpose) {
		if (cublasSetVector(m_M, sizeof(PD::PDScalar), inData, 1, (void*)(m_cudaInVec), 1) != cublasStatus_t::CUBLAS_STATUS_SUCCESS) {
			std::cout << "Fatal error: Could not set data for the in-vector with CUBLAS." << std::endl;
		}
		if (m_massesSize == m_M && m_cudaMassesVec) {
			elementWiseMultiply(m_M, m_cudaInVec, m_cudaMassesVec);
		}
	}
	else {
		if (cublasSetVector(m_N, sizeof(PD::PDScalar), inData, 1, (void*)(m_cudaOutVec), 1) != cublasStatus_t::CUBLAS_STATUS_SUCCESS) {
			std::cout << "Fatal error: Could not set data for the in-vector with CUBLAS." << std::endl;
		}
		if (m_massesSize == m_N && m_cudaMassesVec) {
			elementWiseMultiply(m_N, m_cudaOutVec, m_cudaMassesVec);
		}
	}
	m_setTime.stopStopWatch();

	// If the matrix has more columns then rows, it needs to be transposed before multiplication, since
	// cublasDgemv expects a matrix with more rows than columns...
	m_multTime.startStopWatch();
	if (m_N < m_M) {
		if (!transpose) {
			// In this case a product with the untransposed matrix is desired, however, the matrix stored
			// on the GPU has been transposed before (since it had more columns than rows), so the operation
			// for Dgemv should be OP_T.
			// On the other hand the pointers to cudaInVec and cudaOutVec have the correct sizes (M and N
			// respectively) so that they appear in the normal order.
			// The reasoning for the other three cases below can be deduced from this example.
			if (cublasDgemv(cublasLibHandle, CUBLAS_OP_T, m_M, m_N, &alpha, m_cudaMat, m_M, m_cudaInVec, 1, &cublasZero, m_cudaOutVec, 1) != cublasStatus_t::CUBLAS_STATUS_SUCCESS) {
				std::cout << "Fatal error doing the transposed matrix vector product with CUBLAS." << std::endl;
			}
		}
		else {
			if (cublasDgemv(cublasLibHandle, CUBLAS_OP_N, m_M, m_N, &alpha, m_cudaMat, m_M, m_cudaOutVec, 1, &cublasZero, m_cudaInVec, 1) != cublasStatus_t::CUBLAS_STATUS_SUCCESS) {
				std::cout << "Fatal error doing the transposed matrix vector product with CUBLAS." << std::endl;
			}
		}
	}
	else {
		if (!transpose) {
			if (cublasDgemv(cublasLibHandle, CUBLAS_OP_N, m_N, m_M, &alpha, m_cudaMat, m_N, m_cudaInVec, 1, &cublasZero, m_cudaOutVec, 1) != cublasStatus_t::CUBLAS_STATUS_SUCCESS) {
				std::cout << "Fatal error doing the non-transposed matrix vector product with CUBLAS." << std::endl;
			}
		}
		else {
			if (cublasDgemv(cublasLibHandle, CUBLAS_OP_T, m_N, m_M, &alpha, m_cudaMat, m_N, m_cudaOutVec, 1, &cublasZero, m_cudaInVec, 1) != cublasStatus_t::CUBLAS_STATUS_SUCCESS) {
				std::cout << "Fatal error doing the non-transposed matrix vector product with CUBLAS." << std::endl;
			}
		}
	}
	//cudaDeviceSynchronize();
	m_multTime.stopStopWatch();

	if (outData) {
		m_getTime.startStopWatch();
		if (!transpose) {
			cudaMemcpy(outData, m_cudaOutVec, sizeof(PD::PDScalar) * m_N, cudaMemcpyDeviceToHost);
		}
		else {
			cudaMemcpy(outData, m_cudaInVec, sizeof(PD::PDScalar) * m_M, cudaMemcpyDeviceToHost);
		}
		m_getTime.stopStopWatch();
	}

#ifdef ENABLE_DIRECT_BUFFER_MAP
	if (m_glbufferId > 0 && !m_glArrayPtr) {
		// Hi-jack the buffer from OpenGL
		cudaGraphicsResource_t res;
		glBindBuffer(GL_ARRAY_BUFFER, m_glbufferId);
		cudaGraphicsGLRegisterBuffer(&res, m_glbufferId, cudaGraphicsRegisterFlagsNone);
		cudaGraphicsMapResources(1, &res, 0);
		size_t size;
		cudaGraphicsResourceGetMappedPointer((void**)&m_glArrayPtr, &size, res);
		glBindBuffer(GL_ARRAY_BUFFER, 0);
	}
	if (m_glbufferId > 0 && m_glArrayPtr) {
		//Copy from m_cudaOutVec to m_glArrayPtr while casting from PDScalar to float
		int N = m_N;
		if (cutoff >= 0) {
			N = cutoff;
		}
		doubleToFloatDeviceCpy(N, coord, m_cudaOutVec, m_glArrayPtr);
	}
#endif

	cudaDeviceSynchronize();
}

void CUDAMatrixVectorMultiplier::printTimings()
{
	std::cout << m_setTime.evaluateAverage() << "(" << m_M << ")/" << m_getTime.evaluateAverage() << "(" << m_N << ")/" << m_multTime.evaluateAverage();
}

#ifdef ENABLE_DIRECT_BUFFER_MAP
void CUDAMatrixVectorMultiplier::setGLBuffer(GLUint bufferID)
{
	m_glbufferId = bufferID
}
#endif


CUSparseMatrixVectorMultiplier::CUSparseMatrixVectorMultiplier(PD::PDSparseMatrix & mat)
	: m_alpha(1.), m_zero(0.)
{
	if (!cusparseLibInitialized) {
		cusparseCreate(&cusparseLibHandle);
		cusparseLibInitialized = true;
	}

	m_N = mat.rows();
	m_M = mat.cols();

	if (cudaMalloc((void**)&m_cudaInVec, sizeof(PD::PDScalar)*m_M) != cudaError::cudaSuccess) {
		std::cout << "Fatal error: Could not allocate memory for in-vector in CUDA" << std::endl;
		return;
	}
	if (cudaMalloc((void**)&m_cudaOutVec, sizeof(PD::PDScalar)*m_N) != cudaError::cudaSuccess) {
		std::cout << "Fatal error: Could not allocate memory for out-vector in CUDA" << std::endl;
		return;
	}

	// Create the CSC data for the sparse matrix
	m_nnz = mat.nonZeros();
	PD::PDScalar* entries = new PD::PDScalar[m_nnz];
	int* rowInds = new int[m_nnz];
	int* colPtr = new int[m_M + 1];
	unsigned int counter = 0;
	colPtr[0] = 0;
	for (int k = 0; k < mat.outerSize(); ++k) {
		for (PD::PDSparseMatrix::InnerIterator it(mat, k); it; ++it)
		{
			entries[counter] = it.value();
			rowInds[counter] = it.row();
			counter++;
		}
		colPtr[k + 1] = counter;
	}

	// Upload the CSC data
	if (cudaMalloc((void**)&m_cudaMatData, sizeof(PD::PDScalar)*m_nnz) != cudaError::cudaSuccess) {
		std::cout << "Fatal error: Could not allocate memory for matrix entries in CUDA" << std::endl;
		return;
	}
	else {
		cudaMemcpy(m_cudaMatData, entries, sizeof(PD::PDScalar)*m_nnz, cudaMemcpyHostToDevice);
	}
	if (cudaMalloc((void**)&m_cudaRowInd, sizeof(int)*m_nnz) != cudaError::cudaSuccess) {
		std::cout << "Fatal error: Could not allocate memory for row indices in CUDA" << std::endl;
		return;
	}
	else {
		cudaMemcpy(m_cudaRowInd, rowInds, sizeof(int)*m_nnz, cudaMemcpyHostToDevice);
	}
	if (cudaMalloc((void**)&m_cudaColPtr, sizeof(int)*(m_M + 1)) != cudaError::cudaSuccess) {
		std::cout << "Fatal error: Could not allocate memory for row indices in CUDA" << std::endl;
		return;
	}
	else {
		cudaMemcpy(m_cudaColPtr, colPtr, sizeof(int)*(m_M + 1), cudaMemcpyHostToDevice);
	}

	// Set up description
	cusparseCreateMatDescr(&m_desc);
	cusparseSetMatType(m_desc, CUSPARSE_MATRIX_TYPE_GENERAL);
	cusparseSetMatIndexBase(m_desc, CUSPARSE_INDEX_BASE_ZERO);
}

void CUSparseMatrixVectorMultiplier::mult(const void * inData, void * outData, PD::PDScalar & alpha)
{

	if (cublasSetVector(m_M, sizeof(PD::PDScalar), inData, 1, (void*)(m_cudaInVec), 1) != cublasStatus_t::CUBLAS_STATUS_SUCCESS) {
		std::cout << "Fatal error: Could not set data for the in-vector with CUBLAS." << std::endl;
	}

	cudaMemcpy(m_cudaInVec, inData, sizeof(PD::PDScalar) * m_M, cudaMemcpyHostToDevice);

	/*
		NOTE:
		We do NOT multiply with the transpose of the matrix that was passed to the constructor!!!
		The reason that we set the operation to transpose is that cuSparse only offers a multiplication
		for compressed sparse row (CSR) format type matrices, but the matrix mat is in compressed sparse
		column (CSC) format.
		However, using the CSC description of mat as a CSR description, leads to the transpose of mat.
		Thus, multiplying with the transpose in CSR amounts to multiplying with the non-transposed in
		CSC.
	*/
	cusparseDcsrmv(cusparseLibHandle, CUSPARSE_OPERATION_TRANSPOSE, m_M, m_N, m_nnz, &m_alpha, m_desc, m_cudaMatData, m_cudaColPtr, m_cudaRowInd, m_cudaInVec, &m_zero, m_cudaOutVec);

	cudaMemcpy(outData, m_cudaOutVec, sizeof(PD::PDScalar) * m_N, cudaMemcpyDeviceToHost);
}
