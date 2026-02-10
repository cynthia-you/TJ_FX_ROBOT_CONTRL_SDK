#include "FXMatrix.h"

FX_VOID  FX_Vect3Copy(Vect3 src, Vect3 dst)
{
	FX_INT32 i = 0;
	for (i = 0; i < 3; i++)
	{
		dst[i] = src[i];
	}
}

FX_VOID  FX_Vect4Copy(Vect4 src, Vect4 dst)
{
	FX_INT32 i = 0;
	for (i = 0; i < 4; i++)
	{
		dst[i] = src[i];
	}
}
FX_VOID  FX_Vect6Copy(Vect6 src, Vect6 dst)
{
	FX_INT32 i = 0;
	for (i = 0; i < 6; i++)
	{
		dst[i] = src[i];
	}
}
FX_VOID FX_Vect7Copy(Vect7 src, Vect7 dst)
{
	FX_INT32 i = 0;
	for (i = 0; i < 7; i++)
	{
		dst[i] = src[i];
	}
}
FX_VOID FX_Vect8Copy(Vect8 src, Vect8 dst)
{
	FX_INT32 i = 0;
	for (i = 0; i < 8; i++)
	{
		dst[i] = src[i];
	}
}


FX_VOID	FX_Vect3Sub(Vect3 a, Vect3 b, Vect3 result)
{
	FX_INT32 i;
	for (i = 0; i < 3; i++)
	{
		result[i] = a[i] - b[i];
	}
}

FX_VOID	FX_Vect4Sub(Vect4 a, Vect4 b, Vect4 result)
{
	FX_INT32 i;
	for (i = 0; i < 4; i++)
	{
		result[i] = a[i] - b[i];
	}
}

FX_VOID	FX_Vect6Sub(Vect6 a, Vect6 b, Vect6 result)
{
	FX_INT32 i;
	for (i = 0; i < 6; i++)
	{
		result[i] = a[i] - b[i];
	}
}

FX_VOID	FX_Vect7Sub(Vect7 a, Vect7 b, Vect7 result)
{
	FX_INT32 i;
	for (i = 0; i < 7; i++)
	{
		result[i] = a[i] - b[i];
	}
}

FX_VOID	FX_Vect8Sub(Vect8 a, Vect8 b, Vect8 result)
{
	FX_INT32 i;
	for (i = 0; i < 8; i++)
	{
		result[i] = a[i] - b[i];
	}
}

FX_VOID	FX_Vect3Add(Vect3 a, Vect3 b, Vect3 result)
{
	result[0] = a[0] + b[0];
	result[1] = a[1] + b[1];
	result[2] = a[2] + b[2];
}

FX_VOID	FX_Vect4Add(Vect4 a, Vect4 b, Vect4 result)
{
	result[0] = a[0] + b[0];
	result[1] = a[1] + b[1];
	result[2] = a[2] + b[2];
	result[3] = a[3] + b[3];
}
FX_VOID	FX_Vect6Add(Vect6 a, Vect6 b, Vect6 result)
{
	result[0] = a[0] + b[0];
	result[1] = a[1] + b[1];
	result[2] = a[2] + b[2];
	result[3] = a[3] + b[3];
	result[4] = a[4] + b[4];
	result[5] = a[5] + b[5];
}
FX_VOID	FX_Vect7Add(Vect7 a, Vect7 b, Vect7 result)
{
	result[0] = a[0] + b[0];
	result[1] = a[1] + b[1];
	result[2] = a[2] + b[2];
	result[3] = a[3] + b[3];
	result[4] = a[4] + b[4];
	result[5] = a[5] + b[5];
	result[6] = a[6] + b[6];
}
FX_VOID	FX_Vect8Add(Vect8 a, Vect8 b, Vect8 result)
{
	result[0] = a[0] + b[0];
	result[1] = a[1] + b[1];
	result[2] = a[2] + b[2];
	result[3] = a[3] + b[3];
	result[4] = a[4] + b[4];
	result[5] = a[5] + b[5];
	result[6] = a[6] + b[6];
	result[7] = a[7] + b[7];
}



FX_VOID	FX_Vect3AddToA(Vect3 a, Vect3 b)
{
	a[0] += b[0];
	a[1] += b[1];
	a[2] += b[2];
}
FX_VOID	FX_Vect4AddToA(Vect4 a, Vect4 b)
{
	a[0] += b[0];
	a[1] += b[1];
	a[2] += b[2];
	a[3] += b[3];
}
FX_VOID	FX_Vect6AddToA(Vect6 a, Vect6 b)
{
	a[0] += b[0];
	a[1] += b[1];
	a[2] += b[2];
	a[3] += b[3];
	a[4] += b[4];
	a[5] += b[5];
}
FX_VOID	FX_Vect7AddToA(Vect7 a, Vect7 b)
{
	a[0] += b[0];
	a[1] += b[1];
	a[2] += b[2];
	a[3] += b[3];
	a[4] += b[4];
	a[5] += b[5];
	a[6] += b[6];
}
FX_VOID	FX_Vect8AddToA(Vect8 a, Vect8 b)
{
	a[0] += b[0];
	a[1] += b[1];
	a[2] += b[2];
	a[3] += b[3];
	a[4] += b[4];
	a[5] += b[5];
	a[6] += b[6];
	a[7] += b[7];
}


FX_VOID  FX_IdentM33(Matrix3 m)
{
	FX_INT32 i, j;
	for (i = 0; i < 3; i++) {
		for (j = 0; j < 3; j++) {
			m[i][j] = 0;
		}
		m[i][i] = 1;
	}
}

FX_VOID  FX_IdentM44(Matrix4 m)
{
	FX_INT32 i, j;
	for (i = 0; i < 4; i++) {
		for (j = 0; j < 4; j++) {
			m[i][j] = 0;
		}
		m[i][i] = 1;
	}
}

FX_VOID	FX_IdentM66(Matrix6 m)
{
	FX_INT32 i, j;
	for (i = 0; i < 6; i++) {
		for (j = 0; j < 6; j++) {
			m[i][j] = 0;
		}
		m[i][i] = 1;
	}
}
FX_VOID	FX_IdentM77(Matrix7 m)
{
	FX_INT32 i, j;
	for (i = 0; i < 7; i++) {
		for (j = 0; j < 7; j++) {
			m[i][j] = 0;
		}
		m[i][i] = 1;
	}
}
FX_VOID	FX_IdentM88(Matrix8 m)
{
	FX_INT32 i, j;
	for (i = 0; i < 8; i++) {
		for (j = 0; j < 8; j++) {
			m[i][j] = 0;
		}
		m[i][i] = 1;
	}
}


FX_VOID  FX_M33Copy(Matrix3 src, Matrix3 dst)
{
	FX_INT32L i, j;
	for (i = 0; i < 3; i++)
	{
		for (j = 0; j < 3; j++)
		{
			dst[i][j] = src[i][j];
		}
	}
}
FX_VOID	FX_M44Copy(Matrix4 src, Matrix4 dst)
{
	FX_INT32L i, j;
	for (i = 0; i < 4; i++)
	{
		for (j = 0; j < 4; j++)
		{
			dst[i][j] = src[i][j];
		}
	}
}
FX_VOID	FX_M66Copy(Matrix6 src, Matrix6 dst)
{
	FX_INT32L i, j;
	for (i = 0; i < 6; i++)
	{
		for (j = 0; j < 6; j++)
		{
			dst[i][j] = src[i][j];
		}
	}
}
FX_VOID	FX_M77Copy(Matrix7 src, Matrix7 dst)
{
	FX_INT32L i, j;
	for (i = 0; i < 7; i++)
	{
		for (j = 0; j < 7; j++)
		{
			dst[i][j] = src[i][j];
		}
	}
}
FX_VOID	FX_M88Copy(Matrix8 src, Matrix8 dst)
{
	FX_INT32L i, j;
	for (i = 0; i < 8; i++)
	{
		for (j = 0; j < 8; j++)
		{
			dst[i][j] = src[i][j];
		}
	}
}

FX_VOID	FX_M67Copy(Matrix67 src, Matrix67 dst)
{
	FX_INT32L i, j;
	for (i = 0; i < 6; i++)
	{
		for (j = 0; j < 7; j++)
		{
			dst[i][j] = src[i][j];
		}
	}
}
FX_VOID	FX_M76Copy(Matrix76 src, Matrix76 dst)
{
	FX_INT32L i, j;
	for (i = 0; i < 7; i++)
	{
		for (j = 0; j < 6; j++)
		{
			dst[i][j] = src[i][j];
		}
	}
}

FX_VOID  FX_M33Trans(Matrix3 Org, Matrix3 Result)
{
	FX_INT32L i, j;
	for (i = 0; i < 3; i++)
	{
		for (j = 0; j < 3; j++)
		{
			Result[i][j] = Org[j][i];
		}
	}
}

FX_VOID	FX_M44Trans(Matrix4 Org, Matrix4 Result)
{
	FX_INT32L i, j;
	for (i = 0; i < 4; i++)
	{
		for (j = 0; j < 4; j++)
		{
			Result[i][j] = Org[j][i];
		}
	}
}
FX_VOID	FX_M66Trans(Matrix6 Org, Matrix6 Result)
{
	FX_INT32L i, j;
	for (i = 0; i < 6; i++)
	{
		for (j = 0; j < 6; j++)
		{
			Result[i][j] = Org[j][i];
		}
	}
}
FX_VOID	FX_M77Trans(Matrix7 Org, Matrix7 Result)
{
	FX_INT32L i, j;
	for (i = 0; i < 7; i++)
	{
		for (j = 0; j < 7; j++)
		{
			Result[i][j] = Org[j][i];
		}
	}
}
FX_VOID	FX_M88Trans(Matrix8 Org, Matrix8 Result)
{
	FX_INT32L i, j;
	for (i = 0; i < 8; i++)
	{
		for (j = 0; j < 8; j++)
		{
			Result[i][j] = Org[j][i];
		}
	}
}

FX_VOID	FX_MMV4(Matrix4 L, Vect4 R, Vect4 Result)
{
	FX_INT32L i, k;
	for (i = 0; i < 4; i++)
	{
		Result[i] = 0;
		for (k = 0; k < 4; k++)
		{
			Result[i] += L[i][k] * R[k];
		}
	}
}
FX_VOID	FX_MMV6(Matrix6 L, Vect6 R, Vect6 Result)
{
	FX_INT32L i, k;
	for (i = 0; i < 6; i++)
	{
		Result[i] = 0;
		for (k = 0; k < 6; k++)
		{
			Result[i] += L[i][k] * R[k];
		}
	}
}
FX_VOID	FX_MMV7(Matrix7 L, Vect7 R, Vect7 Result)
{
	FX_INT32L i, k;
	for (i = 0; i < 7; i++)
	{
		Result[i] = 0;
		for (k = 0; k < 7; k++)
		{
			Result[i] += L[i][k] * R[k];
		}
	}
}
FX_VOID	FX_MVM677(Matrix67 L, Vect7 R, Vect6 Result)
{
	FX_INT32L i, k;
	for (i = 0; i < 6; i++)
	{
		Result[i] = 0;
		for (k = 0; k < 7; k++)
		{
			Result[i] += L[i][k] * R[k];
		}
	}
}


FX_VOID	FX_MVM766(Matrix76 L, Vect6 R, Vect7 Result)
{
	FX_INT32L i, k;
	for (i = 0; i < 7; i++)
	{
		Result[i] = 0;
		for (k = 0; k < 6; k++)
		{
			Result[i] += L[i][k] * R[k];
		}
	}
}


FX_VOID	FX_MMV8(Matrix8 L, Vect8 R, Vect8 Result)
{
	FX_INT32L i, k;
	for (i = 0; i < 8; i++)
	{
		Result[i] = 0;
		for (k = 0; k < 8; k++)
		{
			Result[i] += L[i][k] * R[k];
		}
	}
}

FX_VOID	FX_MVM666(Vect6 L, Matrix6 R, Vect6 Result)
{
	FX_INT32L i, k;
	for (i = 0; i < 6; i++)
	{
		Result[i] = 0;
		for (k = 0; k < 6; k++)
		{
			Result[i] += L[k] * R[k][i];
		}
	}
}
FX_VOID	FX_MVM777(Vect7 L, Matrix7 R, Vect7 Result)
{
	FX_INT32L i, k;
	for (i = 0; i < 7; i++)
	{
		Result[i] = 0;
		for (k = 0; k < 7; k++)
		{
			Result[i] += L[k] * R[k][i];
		}
	}
}




FX_VOID FX_MAddM33(Matrix3 L, Matrix3 R, Matrix3 Result)
{
	FX_INT32L i, j;
	for (i = 0; i < 3; i++)
	{
		for (j = 0; j < 3; j++)
		{
			Result[i][j] = L[i][j] + R[i][j];
		}
	}
}
FX_VOID FX_MAddM44(Matrix4 L, Matrix4 R, Matrix4 Result)
{
	FX_INT32L i, j;
	for (i = 0; i < 4; i++)
	{
		for (j = 0; j < 4; j++)
		{
			Result[i][j] = L[i][j] + R[i][j];
		}
	}
}
FX_VOID FX_MAddM66(Matrix6 L, Matrix6 R, Matrix6 Result)
{
	FX_INT32L i, j;
	for (i = 0; i < 6; i++)
	{
		for (j = 0; j < 6; j++)
		{
			Result[i][j] = L[i][j] + R[i][j];
		}
	}
}
FX_VOID FX_MAddM77(Matrix7 L, Matrix7 R, Matrix7 Result)
{
	FX_INT32L i, j;
	for (i = 0; i < 7; i++)
	{
		for (j = 0; j < 7; j++)
		{
			Result[i][j] = L[i][j] + R[i][j];
		}
	}
}
FX_VOID FX_MAddM88(Matrix8 L, Matrix8 R, Matrix8 Result)
{
	FX_INT32L i, j;
	for (i = 0; i < 8; i++)
	{
		for (j = 0; j < 78; j++)
		{
			Result[i][j] = L[i][j] + R[i][j];
		}
	}
}


FX_VOID	FX_MMM33(Matrix3 L, Matrix3 R, Matrix3 Result)
{
	FX_INT32L i, j, k;
	for (i = 0; i < 3; i++)
	{
		for (j = 0; j < 3; j++)
		{
			Result[i][j] = 0;
			for (k = 0; k < 3; k++)
			{
				Result[i][j] += L[i][k] * R[k][j];
			}
		}
	}
}

FX_VOID	FX_MMM44(Matrix4 L, Matrix4 R, Matrix4 Result)
{
	FX_INT32L i, j, k;
	for (i = 0; i < 4; i++)
	{
		for (j = 0; j < 4; j++)
		{
			Result[i][j] = 0;
			for (k = 0; k < 4; k++)
			{
				Result[i][j] += L[i][k] * R[k][j];
			}
		}
	}
}

FX_VOID	FX_MMM66(Matrix6 L, Matrix6 R, Matrix6 Result)
{
	FX_INT32L i, j, k;
	for (i = 0; i < 6; i++)
	{
		for (j = 0; j < 6; j++)
		{
			Result[i][j] = 0;
			for (k = 0; k < 6; k++)
			{
				Result[i][j] += L[i][k] * R[k][j];
			}
		}
	}
}

FX_VOID	FX_MMM77(Matrix7 L, Matrix7 R, Matrix7 Result)
{
	FX_INT32L i, j, k;
	for (i = 0; i < 7; i++)
	{
		for (j = 0; j < 7; j++)
		{
			Result[i][j] = 0;
			for (k = 0; k < 7; k++)
			{
				Result[i][j] += L[i][k] * R[k][j];
			}
		}
	}
}

FX_VOID	FX_MMM88(Matrix8 L, Matrix8 R, Matrix8 Result)
{
	FX_INT32L i, j, k;
	for (i = 0; i < 8; i++)
	{
		for (j = 0; j < 8; j++)
		{
			Result[i][j] = 0;
			for (k = 0; k < 8; k++)
			{
				Result[i][j] += L[i][k] * R[k][j];
			}
		}
	}
}

FX_VOID	FX_MMM6776(Matrix67 L, Matrix76 R, Matrix6 Result)
{
	FX_INT32L i, j, k;
	for (i = 0; i < 6; i++)
	{
		for (j = 0; j < 6; j++)
		{
			Result[i][j] = 0;
			for (k = 0; k < 7; k++)
			{
				Result[i][j] += L[i][k] * R[k][j];
			}
		}
	}
}


FX_VOID	FX_MMM7667(Matrix76 L, Matrix67 R, Matrix7 Result)
{
	FX_INT32L i, j, k;
	for (i = 0; i < 7; i++)
	{
		for (j = 0; j < 7; j++)
		{
			Result[i][j] = 0;
			for (k = 0; k < 6; k++)
			{
				Result[i][j] += L[i][k] * R[k][j];
			}
		}
	}
}

FX_VOID	FX_MMM6777(Matrix67 L, Matrix7 R, Matrix67 Result)
{
	FX_INT32L i, j, k;
	for (i = 0; i < 6; i++)
	{
		for (j = 0; j < 7; j++)
		{
			Result[i][j] = 0;
			for (k = 0; k < 7; k++)
			{
				Result[i][j] += L[i][k] * R[k][j];
			}
		}
	}
}

FX_BOOL	MatrixInv33(Matrix3 orgm, Matrix3 invm)
{
	FX_INT32 m = 3;
	FX_DOUBLE P[10];
	Matrix3 A;
	Vect3   temp;
	FX_M33Copy(orgm, A);
	{
		long i, j, k, imax;
		double maxA, absA;
		for (i = 0; i < m; i++)
		{
			P[i] = i; //Unit permutation matrix, P[m] initialized with m
		}
		P[m] = 0;

		for (i = 0; i < m; i++)
		{
			maxA = 0.0;
			imax = i;

			for (k = i; k < m; k++)
			{
				absA = FX_Fabs(A[k][i]);
				if (absA > maxA)
				{
					maxA = absA;
					imax = k;
				}
			}

			if (maxA <= FXARM_TINYV && maxA >= -FXARM_TINYV)
			{
				return FX_FALSE;
			}

			if (imax != i) {
				//pivoting P
				temp[0] = P[i];
				P[i] = P[imax];
				P[imax] = temp[0];

				//pivoting rows of A
				if (i == 0)
				{
					FX_Vect3Copy(A[imax], temp);
					FX_Vect3Copy(A[0], A[imax]);
					FX_Vect3Copy(temp, A[0]);
				}
				else
				{
					FX_Vect3Copy(A[i], temp);
					FX_Vect3Copy(A[imax], A[i]);
					FX_Vect3Copy(temp, A[imax]);
				}

				P[m]++;
			}

			for (j = i + 1; j < m; j++)
			{
				A[j][i] /= A[i][i];

				for (k = i + 1; k < m; k++)
				{
					A[j][k] -= A[j][i] * A[i][k];
				}
			}

		}
	}
	{
		long i, j, k;
		for (j = 0; j < m; j++)
		{
			for (i = 0; i < m; i++)
			{
				if (P[i] == j)
					invm[i][j] = 1.0;
				else
					invm[i][j] = 0.0;

				for (k = 0; k < i; k++)
					invm[i][j] -= A[i][k] * invm[k][j];
			}

			for (i = m - 1; i >= 0; i--)
			{
				for (k = i + 1; k < m; k++)
					invm[i][j] -= A[i][k] * invm[k][j];

				invm[i][j] = invm[i][j] / A[i][i];
			}
		}
	}
	return FX_TRUE;
}

FX_BOOL	MatrixInv44(Matrix4 orgm, Matrix4 invm)
{
	FX_INT32 m = 4;
	FX_DOUBLE P[10];
	Matrix4 A;
	Vect4   temp;
	FX_M44Copy(orgm, A);
	{
		long i, j, k, imax;
		double maxA, absA;
		for (i = 0; i < m; i++)
		{
			P[i] = i; //Unit permutation matrix, P[m] initialized with m
		}
		P[m] = 0;

		for (i = 0; i < m; i++)
		{
			maxA = 0.0;
			imax = i;

			for (k = i; k < m; k++)
			{
				absA = FX_Fabs(A[k][i]);
				if (absA > maxA)
				{
					maxA = absA;
					imax = k;
				}
			}

			if (maxA <= FXARM_TINYV && maxA >= -FXARM_TINYV)
			{
				return FX_FALSE;
			}

			if (imax != i) {
				//pivoting P
				temp[0] = P[i];
				P[i] = P[imax];
				P[imax] = temp[0];

				//pivoting rows of A
				if (i == 0)
				{
					FX_Vect4Copy(A[imax], temp);
					FX_Vect4Copy(A[0], A[imax]);
					FX_Vect4Copy(temp, A[0]);
				}
				else
				{
					FX_Vect4Copy(A[i], temp);
					FX_Vect4Copy(A[imax], A[i]);
					FX_Vect4Copy(temp, A[imax]);
				}

				P[m]++;
			}

			for (j = i + 1; j < m; j++)
			{
				A[j][i] /= A[i][i];

				for (k = i + 1; k < m; k++)
				{
					A[j][k] -= A[j][i] * A[i][k];
				}
			}

		}
	}
	{
		long i, j, k;
		for (j = 0; j < m; j++)
		{
			for (i = 0; i < m; i++)
			{
				if (P[i] == j)
					invm[i][j] = 1.0;
				else
					invm[i][j] = 0.0;

				for (k = 0; k < i; k++)
					invm[i][j] -= A[i][k] * invm[k][j];
			}

			for (i = m - 1; i >= 0; i--)
			{
				for (k = i + 1; k < m; k++)
					invm[i][j] -= A[i][k] * invm[k][j];

				invm[i][j] = invm[i][j] / A[i][i];
			}
		}
	}
	return FX_TRUE;
}


FX_BOOL	MatrixInv66(Matrix6 orgm, Matrix6 invm)
{
	FX_INT32 m = 6;
	FX_DOUBLE P[10];
	Matrix6 A;
	Vect6   temp;
	FX_M66Copy(orgm, A);
	{
		long i, j, k, imax;
		double maxA, absA;
		for (i = 0; i < m; i++)
		{
			P[i] = i; //Unit permutation matrix, P[m] initialized with m
		}
		P[m] = 0;

		for (i = 0; i < m; i++)
		{
			maxA = 0.0;
			imax = i;

			for (k = i; k < m; k++)
			{
				absA = FX_Fabs(A[k][i]);
				if (absA > maxA)
				{
					maxA = absA;
					imax = k;
				}
			}

			if (maxA <= FXARM_TINYV && maxA >= -FXARM_TINYV)
			{
				return FX_FALSE;
			}

			if (imax != i) {
				//pivoting P
				temp[0] = P[i];
				P[i] = P[imax];
				P[imax] = temp[0];

				//pivoting rows of A
				if (i == 0)
				{
					FX_Vect6Copy(A[imax], temp);
					FX_Vect6Copy(A[0], A[imax]);
					FX_Vect6Copy(temp, A[0]);
				}
				else
				{
					FX_Vect6Copy(A[i], temp);
					FX_Vect6Copy(A[imax], A[i]);
					FX_Vect6Copy(temp, A[imax]);
				}

				P[m]++;
			}

			for (j = i + 1; j < m; j++)
			{
				A[j][i] /= A[i][i];

				for (k = i + 1; k < m; k++)
				{
					A[j][k] -= A[j][i] * A[i][k];
				}
			}

		}
	}
	{
		long i, j, k;
		for (j = 0; j < m; j++)
		{
			for (i = 0; i < m; i++)
			{
				if (P[i] == j)
					invm[i][j] = 1.0;
				else
					invm[i][j] = 0.0;

				for (k = 0; k < i; k++)
					invm[i][j] -= A[i][k] * invm[k][j];
			}

			for (i = m - 1; i >= 0; i--)
			{
				for (k = i + 1; k < m; k++)
					invm[i][j] -= A[i][k] * invm[k][j];

				invm[i][j] = invm[i][j] / A[i][i];
			}
		}
	}
	return FX_TRUE;
}
FX_BOOL	MatrixInv77(Matrix7 orgm, Matrix7 invm)
{
	FX_INT32 m = 7;
	FX_DOUBLE P[10];
	Matrix7 A;
	Vect7   temp;
	FX_M77Copy(orgm, A);
	{
		long i, j, k, imax;
		double maxA, absA;
		for (i = 0; i < m; i++)
		{
			P[i] = i; //Unit permutation matrix, P[m] initialized with m
		}
		P[m] = 0;

		for (i = 0; i < m; i++)
		{
			maxA = 0.0;
			imax = i;

			for (k = i; k < m; k++)
			{
				absA = FX_Fabs(A[k][i]);
				if (absA > maxA)
				{
					maxA = absA;
					imax = k;
				}
			}

			if (maxA <= FXARM_TINYV && maxA >= -FXARM_TINYV)
			{
				return FX_FALSE;
			}

			if (imax != i) {
				//pivoting P
				temp[0] = P[i];
				P[i] = P[imax];
				P[imax] = temp[0];

				//pivoting rows of A
				if (i == 0)
				{
					FX_Vect7Copy(A[imax], temp);
					FX_Vect7Copy(A[0], A[imax]);
					FX_Vect7Copy(temp, A[0]);
				}
				else
				{
					FX_Vect7Copy(A[i], temp);
					FX_Vect7Copy(A[imax], A[i]);
					FX_Vect7Copy(temp, A[imax]);
				}

				P[m]++;
			}

			for (j = i + 1; j < m; j++)
			{
				A[j][i] /= A[i][i];

				for (k = i + 1; k < m; k++)
				{
					A[j][k] -= A[j][i] * A[i][k];
				}
			}

		}
	}
	{
		long i, j, k;
		for (j = 0; j < m; j++)
		{
			for (i = 0; i < m; i++)
			{
				if (P[i] == j)
					invm[i][j] = 1.0;
				else
					invm[i][j] = 0.0;

				for (k = 0; k < i; k++)
					invm[i][j] -= A[i][k] * invm[k][j];
			}

			for (i = m - 1; i >= 0; i--)
			{
				for (k = i + 1; k < m; k++)
					invm[i][j] -= A[i][k] * invm[k][j];

				invm[i][j] = invm[i][j] / A[i][i];
			}
		}
	}
	return FX_TRUE;
}
FX_BOOL	MatrixInv88(Matrix8 orgm, Matrix8 invm)
{
	FX_INT32 m = 8;
	FX_DOUBLE P[10];
	Matrix8 A;
	Vect8   temp;
	FX_M88Copy(orgm, A);
	{
		long i, j, k, imax;
		double maxA, absA;
		for (i = 0; i < m; i++)
		{
			P[i] = i; //Unit permutation matrix, P[m] initialized with m
		}
		P[m] = 0;

		for (i = 0; i < m; i++)
		{
			maxA = 0.0;
			imax = i;

			for (k = i; k < m; k++)
			{
				absA = FX_Fabs(A[k][i]);
				if (absA > maxA)
				{
					maxA = absA;
					imax = k;
				}
			}

			if (maxA <= FXARM_TINYV && maxA >= -FXARM_TINYV)
			{
				return FX_FALSE;
			}

			if (imax != i) {
				//pivoting P
				temp[0] = P[i];
				P[i] = P[imax];
				P[imax] = temp[0];

				//pivoting rows of A
				if (i == 0)
				{
					FX_Vect8Copy(A[imax], temp);
					FX_Vect8Copy(A[0], A[imax]);
					FX_Vect8Copy(temp, A[0]);
				}
				else
				{
					FX_Vect8Copy(A[i], temp);
					FX_Vect8Copy(A[imax], A[i]);
					FX_Vect8Copy(temp, A[imax]);
				}

				P[m]++;
			}

			for (j = i + 1; j < m; j++)
			{
				A[j][i] /= A[i][i];

				for (k = i + 1; k < m; k++)
				{
					A[j][k] -= A[j][i] * A[i][k];
				}
			}

		}
	}
	{
		long i, j, k;
		for (j = 0; j < m; j++)
		{
			for (i = 0; i < m; i++)
			{
				if (P[i] == j)
					invm[i][j] = 1.0;
				else
					invm[i][j] = 0.0;

				for (k = 0; k < i; k++)
					invm[i][j] -= A[i][k] * invm[k][j];
			}

			for (i = m - 1; i >= 0; i--)
			{
				for (k = i + 1; k < m; k++)
					invm[i][j] -= A[i][k] * invm[k][j];

				invm[i][j] = invm[i][j] / A[i][i];
			}
		}
	}
	return FX_TRUE;
}
FX_BOOL	MatrixInvDP33(Matrix3 m, FX_DOUBLE r, Matrix3 invm)
{
	Matrix3 mt;
	Matrix3 a;
	Matrix3 ia;
	FX_INT32L i;
	FX_M33Trans(m, mt);
	FX_MMM33(m, mt, a);
	for (i = 0; i < 3; i++)
	{
		a[i][i] += r;
	}
	if (MatrixInv33(a, ia) == FX_FALSE)
	{
		return FX_FALSE;
	}
	FX_MMM33(mt, ia, invm);
	return FX_TRUE;
}

FX_BOOL	MatrixInvDP44(Matrix4 m, FX_DOUBLE r, Matrix4 invm)
{
	Matrix4 mt;
	Matrix4 a;
	Matrix4 ia;
	FX_INT32L i;
	FX_M44Trans(m, mt);
	FX_MMM44(m, mt, a);
	for (i = 0; i < 4; i++)
	{
		a[i][i] += r;
	}
	if (MatrixInv44(a, ia) == FX_FALSE)
	{
		return FX_FALSE;
	}
	FX_MMM44(mt, ia, invm);
	return FX_TRUE;
}
FX_BOOL	MatrixInvDP66(Matrix6 m, FX_DOUBLE r, Matrix6 invm)
{
	Matrix6 mt;
	Matrix6 a;
	Matrix6 ia;
	FX_INT32L i;
	FX_M66Trans(m, mt);
	FX_MMM66(m, mt, a);
	for (i = 0; i < 6; i++)
	{
		a[i][i] += r;
	}
	if (MatrixInv66(a, ia) == FX_FALSE)
	{
		return FX_FALSE;
	}
	FX_MMM66(mt, ia, invm);
	return FX_TRUE;
}
FX_BOOL	MatrixInvDP77(Matrix7 m, FX_DOUBLE r, Matrix7 invm)
{
	Matrix7 mt;
	Matrix7 a;
	Matrix7 ia;
	FX_INT32L i;
	FX_M77Trans(m, mt);
	FX_MMM77(m, mt, a);
	for (i = 0; i < 7; i++)
	{
		a[i][i] += r;
	}
	if (MatrixInv77(a, ia) == FX_FALSE)
	{
		return FX_FALSE;
	}
	FX_MMM77(mt, ia, invm);
	return FX_TRUE;
}
FX_BOOL	MatrixInvDP88(Matrix8 m, FX_DOUBLE r, Matrix8 invm)
{
	Matrix8 mt;
	Matrix8 a;
	Matrix8 ia;
	FX_INT32L i;
	FX_M88Trans(m, mt);
	FX_MMM88(m, mt, a);
	for (i = 0; i < 8; i++)
	{
		a[i][i] += r;
	}
	if (MatrixInv88(a, ia) == FX_FALSE)
	{
		return FX_FALSE;
	}
	FX_MMM88(mt, ia, invm);
	return FX_TRUE;
}



FX_VOID SVD_CALP(FX_DOUBLE a[], FX_DOUBLE e[], FX_DOUBLE s[], FX_DOUBLE v[], FX_INT32L m, FX_INT32L n)
{
	FX_INT32L i, j, p, q;
	FX_DOUBLE d;
	if (m >= n)
		i = n;
	else
		i = m;
	for (j = 1; j <= i - 1; j++)
	{
		a[(j - 1) * n + j - 1] = s[j - 1];
		a[(j - 1) * n + j] = e[j - 1];
	}
	a[(i - 1) * n + i - 1] = s[i - 1];
	if (m < n)
		a[(i - 1) * n + i] = e[i - 1];
	for (i = 1; i <= n - 1; i++)
		for (j = i + 1; j <= n; j++)
		{
			p = (i - 1) * n + j - 1;
			q = (j - 1) * n + i - 1;
			d = v[p]; v[p] = v[q]; v[q] = d;
		}
	return;
}
FX_VOID SVD_CALS(FX_DOUBLE fg[2], FX_DOUBLE cs[2])
{
	FX_DOUBLE r, d;
	//if((FX_Fabs(fg[0])+FX_Fabs(fg[1]))==0.0)
	if ((FX_Fabs(fg[0]) + FX_Fabs(fg[1])) < FXARM_EPS)
	{
		cs[0] = 1.0; cs[1] = 0.0; d = 0.0;
	}
	else
	{
		d = FX_Sqrt(fg[0] * fg[0] + fg[1] * fg[1]);
		if (FX_Fabs(fg[0]) > FX_Fabs(fg[1]))
		{
			d = FX_Fabs(d);
			if (fg[0] < 0.0)
				d = -d;
		}
		if (FX_Fabs(fg[1]) >= FX_Fabs(fg[0]))
		{
			d = FX_Fabs(d);
			if (fg[1] < 0.0)
				d = -d;
		}
		cs[0] = fg[0] / d;
		cs[1] = fg[1] / d;
	}
	r = 1.0;
	if (FX_Fabs(fg[0]) > FX_Fabs(fg[1]))
		r = cs[1];
	else
		//if(cs[0]!=0.0)
		if (FX_Fabs(cs[0]) > FXARM_EPS)
			r = 1.0 / cs[0];
	fg[0] = d;
	fg[1] = r;
	return;
}

FX_INT32L SVD_UAV(FX_DOUBLE a[], FX_INT32L m, FX_INT32L n, FX_DOUBLE u[], FX_DOUBLE v[], FX_DOUBLE eps, FX_INT32L ka)
{
	FX_INT32L i, j, k, l, it, ll, kk, ix, iy, mm, nn, iz, ml, ks;
	FX_DOUBLE d, dd, t, sm, sml, eml, sk, ek, b, c, shh, fg[2], cs[2];
	FX_DOUBLE s[100], e[100], w[100];

	for (i = 1; i <= m; i++)
	{
		ix = (i - 1) * m + i - 1;
		u[ix] = 0;
	}
	for (i = 1; i <= n; i++)
	{
		iy = (i - 1) * n + i - 1;
		v[iy] = 0;
	}
	it = SVD_MAX_IT_NUM; k = n;
	if (m - 1 < n)
		k = m - 1;
	l = m;
	if (n - 2 < m) l = n - 2;
	if (l < 0) l = 0;
	ll = k;
	if (l > k) ll = l;
	if (ll >= 1)
	{
		for (kk = 1; kk <= ll; kk++)
		{
			if (kk <= k)
			{
				d = 0.0;
				for (i = kk; i <= m; i++)
				{
					ix = (i - 1) * n + kk - 1; d = d + a[ix] * a[ix];
				}
				s[kk - 1] = FX_Sqrt(d);
				//if(s[kk-1]!=0.0)
				if (FX_Fabs(s[kk - 1]) > FXARM_EPS)
				{
					ix = (kk - 1) * n + kk - 1;
					//if(a[ix]!=0.0)
					if (FX_Fabs(a[ix]) > FXARM_EPS)
					{
						s[kk - 1] = FX_Fabs(s[kk - 1]);
						if (a[ix] < 0.0) s[kk - 1] = -s[kk - 1];
					}
					for (i = kk; i <= m; i++)
					{
						iy = (i - 1) * n + kk - 1;
						a[iy] = a[iy] / s[kk - 1];
					}
					a[ix] = 1.0 + a[ix];
				}
				s[kk - 1] = -s[kk - 1];
			}
			if (n >= kk + 1)
			{
				for (j = kk + 1; j <= n; j++)
				{
					//if((kk<=k)&&(s[kk-1]!=0.0))
					if ((kk <= k) && (FX_Fabs(s[kk - 1]) > FXARM_EPS))
					{
						d = 0.0;
						for (i = kk; i <= m; i++)
						{
							ix = (i - 1) * n + kk - 1;
							iy = (i - 1) * n + j - 1;
							d = d + a[ix] * a[iy];
						}
						d = -d / a[(kk - 1) * n + kk - 1];
						for (i = kk; i <= m; i++)
						{
							ix = (i - 1) * n + j - 1;
							iy = (i - 1) * n + kk - 1;
							a[ix] = a[ix] + d * a[iy];
						}
					}
					e[j - 1] = a[(kk - 1) * n + j - 1];
				}
			}
			if (kk <= k)
			{
				for (i = kk; i <= m; i++)
				{
					ix = (i - 1) * m + kk - 1; iy = (i - 1) * n + kk - 1;
					u[ix] = a[iy];
				}
			}
			if (kk <= l)
			{
				d = 0.0;
				for (i = kk + 1; i <= n; i++)
					d = d + e[i - 1] * e[i - 1];
				e[kk - 1] = FX_Sqrt(d);
				//if(e[kk-1]!=0.0)
				if (FX_Fabs(e[kk - 1]) > FXARM_EPS)
				{
					//if(e[kk]!=0.0)
					if (FX_Fabs(e[kk]) > FXARM_EPS)
					{
						e[kk - 1] = FX_Fabs(e[kk - 1]);
						if (e[kk] < 0.0)
							e[kk - 1] = -e[kk - 1];
					}
					for (i = kk + 1; i <= n; i++)
						e[i - 1] = e[i - 1] / e[kk - 1];
					e[kk] = 1.0 + e[kk];
				}
				e[kk - 1] = -e[kk - 1];
				//if((kk+1<=m)&&(e[kk-1]!=0.0))
				if ((kk + 1 <= m) && (FX_Fabs(e[kk - 1]) > FXARM_EPS))
				{
					for (i = kk + 1; i <= m; i++) w[i - 1] = 0.0;
					for (j = kk + 1; j <= n; j++)
						for (i = kk + 1; i <= m; i++)
							w[i - 1] = w[i - 1] + e[j - 1] * a[(i - 1) * n + j - 1];
					for (j = kk + 1; j <= n; j++)
						for (i = kk + 1; i <= m; i++)
						{
							ix = (i - 1) * n + j - 1;
							a[ix] = a[ix] - w[i - 1] * e[j - 1] / e[kk];
						}
				}
				for (i = kk + 1; i <= n; i++)
					v[(i - 1) * n + kk - 1] = e[i - 1];
			}
		}
	}
	mm = n;
	if (m + 1 < n) mm = m + 1;
	if (k < n) s[k] = a[k * n + k];
	if (m < mm) s[mm - 1] = 0.0;
	if (l + 1 < mm) e[l] = a[l * n + mm - 1];
	e[mm - 1] = 0.0;
	nn = m;
	if (m > n) nn = n;
	if (nn >= k + 1)
	{
		for (j = k + 1; j <= nn; j++)
		{
			for (i = 1; i <= m; i++)
				u[(i - 1) * m + j - 1] = 0.0;
			u[(j - 1) * m + j - 1] = 1.0;
		}
	}
	if (k >= 1)/////////////////////////////////
	{
		for (ll = 1; ll <= k; ll++)
		{
			kk = k - ll + 1; iz = (kk - 1) * m + kk - 1;
			//if(s[kk-1]!=0.0)
			if (FX_Fabs(s[kk - 1]) > FXARM_EPS)
			{
				if (nn >= kk + 1)
					for (j = kk + 1; j <= nn; j++)
					{
						d = 0.0;
						for (i = kk; i <= m; i++)
						{
							ix = (i - 1) * m + kk - 1;
							iy = (i - 1) * m + j - 1;
							d = d + u[ix] * u[iy] / u[iz];
						}
						d = -d;
						for (i = kk; i <= m; i++)
						{
							ix = (i - 1) * m + j - 1;
							iy = (i - 1) * m + kk - 1;
							u[ix] = u[ix] + d * u[iy];
						}
					}
				for (i = kk; i <= m; i++)
				{
					ix = (i - 1) * m + kk - 1;
					u[ix] = -u[ix];
				}
				u[iz] = 1.0 + u[iz];
				if (kk - 1 >= 1)//////////////////////////////////////
					for (i = 1; i <= kk - 1; i++)
						u[(i - 1) * m + kk - 1] = 0.0;
			}
			else
			{
				for (i = 1; i <= m; i++)
					u[(i - 1) * m + kk - 1] = 0.0;
				u[(kk - 1) * m + kk - 1] = 1.0;
			}
		}
	}
	for (ll = 1; ll <= n; ll++)
	{
		kk = n - ll + 1; iz = kk * n + kk - 1;
		//if((kk<=l)&&(e[kk-1]!=0.0))/////////////////////////////
		if ((kk <= l) && (FX_Fabs(e[kk - 1]) > FXARM_EPS))
		{
			for (j = kk + 1; j <= n; j++)
			{
				d = 0.0;
				for (i = kk + 1; i <= n; i++)
				{
					ix = (i - 1) * n + kk - 1; iy = (i - 1) * n + j - 1;
					d = d + v[ix] * v[iy] / v[iz];
				}
				d = -d;
				for (i = kk + 1; i <= n; i++)
				{
					ix = (i - 1) * n + j - 1; iy = (i - 1) * n + kk - 1;
					v[ix] = v[ix] + d * v[iy];
				}
			}
		}
		for (i = 1; i <= n; i++)
			v[(i - 1) * n + kk - 1] = 0.0;
		v[iz - n] = 1.0;
	}
	for (i = 1; i <= m; i++)
		for (j = 1; j <= n; j++)
			a[(i - 1) * n + j - 1] = 0.0;
	ml = mm;
	it = SVD_MAX_IT_NUM;
	while (1 == 1)//////////////////////////////////
	{
		if (mm == 0)
		{
			SVD_CALP(a, e, s, v, m, n);
			return l;
		}
		if (it == 0)
		{
			SVD_CALP(a, e, s, v, m, n);
			return -1;
		}
		kk = mm - 1;
		while ((kk != 0) && (FX_Fabs(e[kk - 1]) > FXARM_EPS))
		{
			d = FX_Fabs(s[kk - 1]) + FX_Fabs(s[kk]);
			dd = FX_Fabs(e[kk - 1]);
			if (dd > eps * d)
				kk = kk - 1;
			else
				e[kk - 1] = 0.0;
		}
		if (kk == mm - 1)
		{
			kk = kk + 1;
			if (s[kk - 1] < 0.0)
			{
				s[kk - 1] = -s[kk - 1];
				for (i = 1; i <= n; i++)
				{
					ix = (i - 1) * n + kk - 1;
					v[ix] = -v[ix];
				}
			}
			while ((kk != ml) && (s[kk - 1] < s[kk]))
			{
				d = s[kk - 1]; s[kk - 1] = s[kk]; s[kk] = d;
				if (kk < n)
					for (i = 1; i <= n; i++)
					{
						ix = (i - 1) * n + kk - 1; iy = (i - 1) * n + kk;
						d = v[ix]; v[ix] = v[iy]; v[iy] = d;
					}
				if (kk < m)
					for (i = 1; i <= m; i++)
					{
						ix = (i - 1) * m + kk - 1;
						iy = (i - 1) * m + kk;
						d = u[ix]; u[ix] = u[iy]; u[iy] = d;
					}
				kk = kk + 1;
			}
			it = SVD_MAX_IT_NUM;
			mm = mm - 1;
		}
		else
		{
			ks = mm;
			while ((ks > kk) && (FX_Fabs(s[ks - 1]) > FXARM_EPS))
			{
				d = 0.0;
				if (ks != mm)
					d = d + FX_Fabs(e[ks - 1]);
				if (ks != kk + 1) d = d + FX_Fabs(e[ks - 2]);
				dd = FX_Fabs(s[ks - 1]);
				if (dd > eps * d)
					ks = ks - 1;
				else
					s[ks - 1] = 0.0;
			}
			if (ks == kk)
			{
				kk = kk + 1;
				d = FX_Fabs(s[mm - 1]);
				t = FX_Fabs(s[mm - 2]);
				if (t > d)
					d = t;
				t = FX_Fabs(e[mm - 2]);
				if (t > d)
					d = t;
				t = FX_Fabs(s[kk - 1]);
				if (t > d)
					d = t;
				t = FX_Fabs(e[kk - 1]);
				if (t > d)
					d = t;
				sm = s[mm - 1] / d; sml = s[mm - 2] / d;
				eml = e[mm - 2] / d;
				sk = s[kk - 1] / d; ek = e[kk - 1] / d;
				b = ((sml + sm) * (sml - sm) + eml * eml) / 2.0;
				c = sm * eml; c = c * c; shh = 0.0;
				//if((b!=0.0)||(c!=0.0))
				if ((FX_Fabs(b) > FXARM_EPS) || (FX_Fabs(c) > FXARM_EPS))
				{
					shh = FX_Sqrt(b * b + c);
					if (b < 0.0)
						shh = -shh;
					shh = c / (b + shh);
				}
				fg[0] = (sk + sm) * (sk - sm) - shh;
				fg[1] = sk * ek;
				for (i = kk; i <= mm - 1; i++)
				{
					SVD_CALS(fg, cs);
					if (i != kk)
						e[i - 2] = fg[0];
					fg[0] = cs[0] * s[i - 1] + cs[1] * e[i - 1];
					e[i - 1] = cs[0] * e[i - 1] - cs[1] * s[i - 1];
					fg[1] = cs[1] * s[i];
					s[i] = cs[0] * s[i];
					//if((cs[0]!=1.0)||(cs[1]!=0.0))
					if ((FX_Fabs(cs[0] - 1.0) > FXARM_EPS) || (FX_Fabs(cs[1]) > FXARM_EPS))
						for (j = 1; j <= n; j++)
						{
							ix = (j - 1) * n + i - 1;
							iy = (j - 1) * n + i;
							d = cs[0] * v[ix] + cs[1] * v[iy];
							v[iy] = -cs[1] * v[ix] + cs[0] * v[iy];
							v[ix] = d;
						}
					SVD_CALS(fg, cs);
					s[i - 1] = fg[0];
					fg[0] = cs[0] * e[i - 1] + cs[1] * s[i];
					s[i] = -cs[1] * e[i - 1] + cs[0] * s[i];
					fg[1] = cs[1] * e[i];
					e[i] = cs[0] * e[i];
					if (i < m)
						//if((cs[0]!=1.0)||(cs[1]!=0.0))
						if ((FX_Fabs(cs[0] - 1.0) > FXARM_EPS) || (FX_Fabs(cs[1]) > FXARM_EPS))
							for (j = 1; j <= m; j++)
							{
								ix = (j - 1) * m + i - 1;
								iy = (j - 1) * m + i;
								d = cs[0] * u[ix] + cs[1] * u[iy];
								u[iy] = -cs[1] * u[ix] + cs[0] * u[iy];
								u[ix] = d;
							}
				}
				e[mm - 2] = fg[0];
				it = it - 1;
			}
			else
			{
				if (ks == mm)
				{
					kk = kk + 1;
					fg[1] = e[mm - 2]; e[mm - 2] = 0.0;
					for (ll = kk; ll <= mm - 1; ll++)
					{
						i = mm + kk - ll - 1;
						fg[0] = s[i - 1];
						SVD_CALS(fg, cs);
						s[i - 1] = fg[0];
						if (i != kk)
						{
							fg[1] = -cs[1] * e[i - 2];
							e[i - 2] = cs[0] * e[i - 2];
						}
						//if((cs[0]!=1.0)||(cs[1]!=0.0))
						if ((FX_Fabs(cs[0] - 1.0) > FXARM_EPS) || (FX_Fabs(cs[1]) > FXARM_EPS))
							for (j = 1; j <= n; j++)
							{
								ix = (j - 1) * n + i - 1;
								iy = (j - 1) * n + mm - 1;
								d = cs[0] * v[ix] + cs[1] * v[iy];
								v[iy] = -cs[1] * v[ix] + cs[0] * v[iy];
								v[ix] = d;
							}
					}
				}
				else
				{
					kk = ks + 1;
					fg[1] = e[kk - 2];
					e[kk - 2] = 0.0;
					for (i = kk; i <= mm; i++)
					{
						fg[0] = s[i - 1];
						SVD_CALS(fg, cs);
						s[i - 1] = fg[0];
						fg[1] = -cs[1] * e[i - 1];
						e[i - 1] = cs[0] * e[i - 1];
						//if((cs[0]!=1.0)||(cs[1]!=0.0))
						if ((FX_Fabs(cs[0] - 1.0) > FXARM_EPS) || (FX_Fabs(cs[1]) > FXARM_EPS))
							for (j = 1; j <= m; j++)
							{
								ix = (j - 1) * m + i - 1;
								iy = (j - 1) * m + kk - 2;
								d = cs[0] * u[ix] + cs[1] * u[iy];
								u[iy] = -cs[1] * u[ix] + cs[0] * u[iy];
								u[ix] = d;
							}
					}
				}
			}
		}
	}
	return l;
}


FX_BOOL FX_SVDM_33(Matrix3 mo, Matrix3 u, Matrix3 s, Matrix3 v)
{
	FX_DOUBLE ss[100];
	FX_DOUBLE su[100];
	FX_DOUBLE sv[100];
	FX_INT32L i;
	FX_INT32L j;
	FX_INT32L m;
	FX_INT32L n;
	FX_INT32L ms;
	m = 3;
	n = 3;

	ms = m;
	if (n > m)
	{
		ms = n;
	}
	ms += 1;

	for (i = 0; i < 100; i++)
	{
		ss[i] = 0;
		su[i] = 0;
		sv[i] = 0;
	}
	for (i = 0; i < m; i++)
	{
		for (j = 0; j < n; j++)
		{
			ss[i * n + j] = mo[i][j];
		}
	}

	if (SVD_UAV(ss, m, n, su, sv, FXARM_EPS, ms) < 0)
	{
		return FX_FALSE;
	}

	for (i = 0; i < m; i++)
	{
		for (j = 0; j < n; j++)
		{
			s[i][j] = ss[i * n + j];
		}
	}


	for (i = 0; i < m; i++)
	{
		for (j = 0; j < m; j++)
		{
			u[i][j] = su[i * m + j];
		}
	}

	for (i = 0; i < n; i++)
	{
		for (j = 0; j < n; j++)
		{
			v[i][j] = sv[i * n + j];
		}
	}
	return FX_TRUE;
}

FX_BOOL FX_SVDM_44(Matrix4 mo, Matrix4 u, Matrix4 s, Matrix4 v)
{
	FX_DOUBLE ss[100];
	FX_DOUBLE su[100];
	FX_DOUBLE sv[100];
	FX_INT32L i;
	FX_INT32L j;
	FX_INT32L m;
	FX_INT32L n;
	FX_INT32L ms;
	m = 4;
	n = 4;

	ms = m;
	if (n > m)
	{
		ms = n;
	}
	ms += 1;

	for (i = 0; i < 100; i++)
	{
		ss[i] = 0;
		su[i] = 0;
		sv[i] = 0;
	}
	for (i = 0; i < m; i++)
	{
		for (j = 0; j < n; j++)
		{
			ss[i * n + j] = mo[i][j];
		}
	}

	if (SVD_UAV(ss, m, n, su, sv, FXARM_EPS, ms) < 0)
	{
		return FX_FALSE;
	}

	for (i = 0; i < m; i++)
	{
		for (j = 0; j < n; j++)
		{
			s[i][j] = ss[i * n + j];
		}
	}


	for (i = 0; i < m; i++)
	{
		for (j = 0; j < m; j++)
		{
			u[i][j] = su[i * m + j];
		}
	}

	for (i = 0; i < n; i++)
	{
		for (j = 0; j < n; j++)
		{
			v[i][j] = sv[i * n + j];
		}
	}
	return FX_TRUE;
}
FX_BOOL FX_SVDM_66(Matrix6 mo, Matrix6 u, Matrix6 s, Matrix6 v)
{
	FX_DOUBLE ss[100];
	FX_DOUBLE su[100];
	FX_DOUBLE sv[100];
	FX_INT32L i;
	FX_INT32L j;
	FX_INT32L m;
	FX_INT32L n;
	FX_INT32L ms;
	m = 6;
	n = 6;

	ms = m;
	if (n > m)
	{
		ms = n;
	}
	ms += 1;

	for (i = 0; i < 100; i++)
	{
		ss[i] = 0;
		su[i] = 0;
		sv[i] = 0;
	}
	for (i = 0; i < m; i++)
	{
		for (j = 0; j < n; j++)
		{
			ss[i * n + j] = mo[i][j];
		}
	}

	if (SVD_UAV(ss, m, n, su, sv, FXARM_EPS, ms) < 0)
	{
		return FX_FALSE;
	}

	for (i = 0; i < m; i++)
	{
		for (j = 0; j < n; j++)
		{
			s[i][j] = ss[i * n + j];
		}
	}


	for (i = 0; i < m; i++)
	{
		for (j = 0; j < m; j++)
		{
			u[i][j] = su[i * m + j];
		}
	}

	for (i = 0; i < n; i++)
	{
		for (j = 0; j < n; j++)
		{
			v[i][j] = sv[i * n + j];
		}
	}
	return FX_TRUE;
}
FX_BOOL FX_SVDM_77(Matrix7 mo, Matrix7 u, Matrix7 s, Matrix7 v)
{
	FX_DOUBLE ss[100];
	FX_DOUBLE su[100];
	FX_DOUBLE sv[100];
	FX_INT32L i;
	FX_INT32L j;
	FX_INT32L m;
	FX_INT32L n;
	FX_INT32L ms;
	m = 7;
	n = 7;

	ms = m;
	if (n > m)
	{
		ms = n;
	}
	ms += 1;

	for (i = 0; i < 100; i++)
	{
		ss[i] = 0;
		su[i] = 0;
		sv[i] = 0;
	}
	for (i = 0; i < m; i++)
	{
		for (j = 0; j < n; j++)
		{
			ss[i * n + j] = mo[i][j];
		}
	}

	if (SVD_UAV(ss, m, n, su, sv, FXARM_EPS, ms) < 0)
	{
		return FX_FALSE;
	}

	for (i = 0; i < m; i++)
	{
		for (j = 0; j < n; j++)
		{
			s[i][j] = ss[i * n + j];
		}
	}


	for (i = 0; i < m; i++)
	{
		for (j = 0; j < m; j++)
		{
			u[i][j] = su[i * m + j];
		}
	}

	for (i = 0; i < n; i++)
	{
		for (j = 0; j < n; j++)
		{
			v[i][j] = sv[i * n + j];
		}
	}
	return FX_TRUE;
}
FX_BOOL FX_SVDM_88(Matrix8 mo, Matrix8 u, Matrix8 s, Matrix8 v)
{
	FX_DOUBLE ss[100];
	FX_DOUBLE su[100];
	FX_DOUBLE sv[100];
	FX_INT32L i;
	FX_INT32L j;
	FX_INT32L m;
	FX_INT32L n;
	FX_INT32L ms;
	m = 8;
	n = 8;

	ms = m;
	if (n > m)
	{
		ms = n;
	}
	ms += 1;

	for (i = 0; i < 100; i++)
	{
		ss[i] = 0;
		su[i] = 0;
		sv[i] = 0;
	}
	for (i = 0; i < m; i++)
	{
		for (j = 0; j < n; j++)
		{
			ss[i * n + j] = mo[i][j];
		}
	}

	if (SVD_UAV(ss, m, n, su, sv, FXARM_EPS, ms) < 0)
	{
		return FX_FALSE;
	}

	for (i = 0; i < m; i++)
	{
		for (j = 0; j < n; j++)
		{
			s[i][j] = ss[i * n + j];
		}
	}


	for (i = 0; i < m; i++)
	{
		for (j = 0; j < m; j++)
		{
			u[i][j] = su[i * m + j];
		}
	}

	for (i = 0; i < n; i++)
	{
		for (j = 0; j < n; j++)
		{
			v[i][j] = sv[i * n + j];
		}
	}
	return FX_TRUE;
}


FX_VOID FX_UTM_33(Matrix3 m)
{
	FX_INT32 i;
	FX_INT32 j;
	FX_INT32 k;
	FX_INT32 dimension = 3;

	for (i = 0; i < dimension; i++)
	{
		for (j = i + 1; j < dimension; j++)
		{
			for (k = dimension - 1; k >= i; k--)
			{
				m[j][k] -= m[i][k] * m[j][i] / m[i][i];
			}
		}
	}
}
FX_VOID FX_UTM_44(Matrix4 m)
{
	FX_INT32 i;
	FX_INT32 j;
	FX_INT32 k;
	FX_INT32 dimension = 4;

	for (i = 0; i < dimension; i++)
	{
		for (j = i + 1; j < dimension; j++)
		{
			for (k = dimension - 1; k >= i; k--)
			{
				m[j][k] -= m[i][k] * m[j][i] / m[i][i];
			}
		}
	}
}
FX_VOID FX_UTM_66(Matrix6 m)
{
	FX_INT32 i;
	FX_INT32 j;
	FX_INT32 k;
	FX_INT32 dimension = 6;

	for (i = 0; i < dimension; i++)
	{
		for (j = i + 1; j < dimension; j++)
		{
			for (k = dimension - 1; k >= i; k--)
			{
				m[j][k] -= m[i][k] * m[j][i] / m[i][i];
			}
		}
	}
}
FX_VOID FX_UTM_77(Matrix7 m)
{
	FX_INT32 i;
	FX_INT32 j;
	FX_INT32 k;
	FX_INT32 dimension = 7;

	for (i = 0; i < dimension; i++)
	{
		for (j = i + 1; j < dimension; j++)
		{
			for (k = dimension - 1; k >= i; k--)
			{
				m[j][k] -= m[i][k] * m[j][i] / m[i][i];
			}
		}
	}
}
FX_VOID FX_UTM_88(Matrix8 m)
{
	FX_INT32 i;
	FX_INT32 j;
	FX_INT32 k;
	FX_INT32 dimension = 8;

	for (i = 0; i < dimension; i++)
	{
		for (j = i + 1; j < dimension; j++)
		{
			for (k = dimension - 1; k >= i; k--)
			{
				m[j][k] -= m[i][k] * m[j][i] / m[i][i];
			}
		}
	}
}


FX_DOUBLE FX_DetM_33(Matrix3 m)
{
	FX_INT32 i;
	FX_DOUBLE ret = 1;
	FX_UTM_33(m);
	for (i = 0; i < 3; i++)
	{
		ret *= m[i][i];
	}
	return ret;
}

FX_DOUBLE FX_DetM_44(Matrix4 m)
{
	FX_INT32 i;
	FX_DOUBLE ret = 1;
	FX_UTM_44(m);
	for (i = 0; i < 4; i++)
	{
		ret *= m[i][i];
	}
	return ret;
}

FX_DOUBLE FX_DetM_66(Matrix6 m)
{
	FX_INT32 i;
	FX_DOUBLE ret = 1;
	FX_UTM_66(m);
	for (i = 0; i < 6; i++)
	{
		ret *= m[i][i];
	}
	return ret;
}

FX_DOUBLE FX_DetM_77(Matrix7 m)
{
	FX_INT32 i;
	FX_DOUBLE ret = 1;
	FX_UTM_77(m);
	for (i = 0; i < 7; i++)
	{
		ret *= m[i][i];
	}
	return ret;
}

FX_DOUBLE FX_DetM_88(Matrix8 m)
{
	FX_INT32 i;
	FX_DOUBLE ret = 1;
	FX_UTM_88(m);
	for (i = 0; i < 8; i++)
	{
		ret *= m[i][i];
	}
	return ret;
}


FX_DOUBLE FX_VectDot3(Vect3 a, Vect3 b)
{
	FX_INT32 i = 0;
	FX_DOUBLE ret = 0;
	for ( i = 0; i < 3; i++)
	{
		ret += a[i] * b[i];
	}
	return ret;
}
FX_DOUBLE FX_VectDot4(Vect4 a, Vect4 b)
{
	FX_INT32 i = 0;
	FX_DOUBLE ret = 0;
	for (i = 0; i < 4; i++)
	{
		ret += a[i] * b[i];
	}
	return ret;
}
FX_DOUBLE FX_VectDot6(Vect6 a, Vect6 b)
{
	FX_INT32 i = 0;
	FX_DOUBLE ret = 0;
	for (i = 0; i < 6; i++)
	{
		ret += a[i] * b[i];
	}
	return ret;
}
FX_DOUBLE FX_VectDot7(Vect7 a, Vect7 b)
{
	FX_INT32 i = 0;
	FX_DOUBLE ret = 0;
	for (i = 0; i < 7; i++)
	{
		ret += a[i] * b[i];
	}
	return ret;
}
FX_DOUBLE FX_VectDot8(Vect8 a, Vect8 b)
{
	FX_INT32 i = 0;
	FX_DOUBLE ret = 0;
	for (i = 0; i < 8; i++)
	{
		ret += a[i] * b[i];
	}
	return ret;
}


FX_VOID LDLT(Matrix7 A, Matrix7 L , Vect7 D)
{
	FX_INT32 DOF = 7;
	FX_INT32 n = DOF;
	FX_INT32 i, j, h;
	FX_DOUBLE temp = 0.0;
	FX_DOUBLE sum = 0.0;
	D[0] = A[0][0];
	for (i = 1; i < n; i++) {
		for (j = 0; j < i; j++) {
			if (j == 0) {
				L[i][j] = A[i][j] / D[j];
			}
			else {
				for (h = 0; h < j; h++) {
					temp = L[i][h] * L[j][h] * D[h];
					sum += temp;
				}
				L[i][j] = (A[i][j] - sum) / D[j];
				sum = 0.0;
			}
		}
		for (h = 0; h < i; h++) {
			temp = L[i][h] * L[i][h] * D[h];
			sum += temp;
		}
		D[i] = A[i][i] - sum;
		sum = 0.0;
	}
}


FX_VOID FX_SPMatInv77(Matrix7 orgm , Matrix7 invm )
{
	FX_INT32 DOF = 7;
	FX_INT32 i, j, k;
	Matrix7 L = { 0 };
	Vect7  D = { 0 };
	Matrix7 Linv = { 0 };
	Matrix7 LTinv = { 0 };
	for (i = 0; i < DOF; i++)
	{
		L[i][i] = 1.0;
		Linv[i][i] = 1.0;
	}
	LDLT(orgm, L, D);
	for (i = 0; i < DOF; i++) {
		D[i] = 1 / D[i];
	}
	for (i = 0; i < DOF; ++i) {
		for (j = 0; j < i; ++j) {
			FX_DOUBLE sum = 0.0;
			for (k = j; k < i; ++k) {
				sum += L[i][k] * Linv[k][j];
			}
			Linv[i][j] = -sum / L[i][i];
		}
	}
	for (i = 0; i < DOF; i++) {
		LTinv[i][i] = D[i];
	}
	for (i = DOF - 1; i > 0; i--) {
		for (j = DOF - i; j < DOF; j++) {
			LTinv[DOF - 1 - i][j] = Linv[j][DOF - 1 - i] * D[j];
		}
	}
	FX_MMM77(LTinv, Linv, invm);
}



FX_BOOL FX_MatrixNormZX(Vect3 z, Vect3 x, Matrix3 ret_m)
{
	Vect3 zdir;
	Vect3 ydir;
	Vect3 xdir;

	FX_Vect3Copy(z, zdir);
	FX_Vect3Copy(x, xdir);

	if (FX_VectNorm(zdir) == FX_FALSE) { return FX_FALSE; }
	if (FX_VectNorm(xdir) == FX_FALSE) { return FX_FALSE; }
	FX_VectCross(zdir, xdir, ydir);
	if (FX_VectNorm(ydir) == FX_FALSE) { return FX_FALSE; }
	FX_VectCross(ydir, zdir, xdir);
	if (FX_VectNorm(xdir) == FX_FALSE) { return FX_FALSE; }

	ret_m[0][0] = xdir[0];
	ret_m[1][0] = xdir[1];
	ret_m[2][0] = xdir[2];

	ret_m[0][1] = ydir[0];
	ret_m[1][1] = ydir[1];
	ret_m[2][1] = ydir[2];

	ret_m[0][2] = zdir[0];
	ret_m[1][2] = zdir[1];
	ret_m[2][2] = zdir[2];

	return FX_TRUE;
}

FX_BOOL FX_VectNorm(Vect3 a)
{

	FX_DOUBLE r = FX_Sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
	if (r < FXARM_MICRO)
	{
		return FX_FALSE;
	}
	a[0] /= r;
	a[1] /= r;
	a[2] /= r;
	return FX_TRUE;
}

FX_VOID FX_VectCross(Vect3 a, Vect3 b, Vect3 result)
{
	result[0] = a[1] * b[2] - a[2] * b[1];
	result[1] = a[2] * b[0] - a[0] * b[2];
	result[2] = a[0] * b[1] - a[1] * b[0];
}


FX_VOID FX_PGPointMap(Matrix4 a_to_b, Vect3 in_a, Vect3 ret_in_b)
{
	ret_in_b[0] = a_to_b[0][3] + a_to_b[0][0] * in_a[0] + a_to_b[0][1] * in_a[1] + a_to_b[0][2] * in_a[2];
	ret_in_b[1] = a_to_b[1][3] + a_to_b[1][0] * in_a[0] + a_to_b[1][1] * in_a[1] + a_to_b[1][2] * in_a[2];
	ret_in_b[2] = a_to_b[2][3] + a_to_b[2][0] * in_a[0] + a_to_b[2][1] * in_a[1] + a_to_b[2][2] * in_a[2];
}

FX_VOID	FX_PGVectMap(Matrix4 a_to_b, Vect3 in_a, Vect3 ret_in_b)
{
	ret_in_b[0] = a_to_b[0][0] * in_a[0] + a_to_b[0][1] * in_a[1] + a_to_b[0][2] * in_a[2];
	ret_in_b[1] = a_to_b[1][0] * in_a[0] + a_to_b[1][1] * in_a[1] + a_to_b[1][2] * in_a[2];
	ret_in_b[2] = a_to_b[2][0] * in_a[0] + a_to_b[2][1] * in_a[1] + a_to_b[2][2] * in_a[2];
}


FX_VOID	FX_PGVectMapInv(Matrix4 a_to_b, Vect3 ret_in_a, Vect3 in_b)
{
	ret_in_a[0] = a_to_b[0][0] * in_b[0] + a_to_b[1][0] * in_b[1] + a_to_b[2][0] * in_b[2];
	ret_in_a[1] = a_to_b[0][1] * in_b[0] + a_to_b[1][1] * in_b[1] + a_to_b[2][1] * in_b[2];
	ret_in_a[2] = a_to_b[0][2] * in_b[0] + a_to_b[1][2] * in_b[1] + a_to_b[2][2] * in_b[2];
}


FX_VOID  FX_PGMult(Matrix4 L, Matrix4 R, Matrix4 Result)
{
	Result[0][0] = L[0][0] * R[0][0] + L[0][1] * R[1][0] + L[0][2] * R[2][0];
	Result[0][1] = L[0][0] * R[0][1] + L[0][1] * R[1][1] + L[0][2] * R[2][1];
	Result[0][2] = L[0][0] * R[0][2] + L[0][1] * R[1][2] + L[0][2] * R[2][2];

	Result[1][0] = L[1][0] * R[0][0] + L[1][1] * R[1][0] + L[1][2] * R[2][0];
	Result[1][1] = L[1][0] * R[0][1] + L[1][1] * R[1][1] + L[1][2] * R[2][1];
	Result[1][2] = L[1][0] * R[0][2] + L[1][1] * R[1][2] + L[1][2] * R[2][2];

	Result[2][0] = L[2][0] * R[0][0] + L[2][1] * R[1][0] + L[2][2] * R[2][0];
	Result[2][1] = L[2][0] * R[0][1] + L[2][1] * R[1][1] + L[2][2] * R[2][1];
	Result[2][2] = L[2][0] * R[0][2] + L[2][1] * R[1][2] + L[2][2] * R[2][2];

	Result[0][3] = L[0][0] * R[0][3] + L[0][1] * R[1][3] + L[0][2] * R[2][3] + L[0][3];
	Result[1][3] = L[1][0] * R[0][3] + L[1][1] * R[1][3] + L[1][2] * R[2][3] + L[1][3];
	Result[2][3] = L[2][0] * R[0][3] + L[2][1] * R[1][3] + L[2][2] * R[2][3] + L[2][3];

	Result[3][0] = 0;
	Result[3][1] = 0;
	Result[3][2] = 0;
	Result[3][3] = 1;
}
FX_VOID	FX_PGGetGes(PosGes src, Matrix3 ges)
{
	FX_INT32L i, j;
	for (i = 0; i < 3; i++)
	{
		for (j = 0; j < 3; j++)
		{
			ges[i][j] = src[i][j];
		}
	}
}

FX_VOID	FX_PGGetPos(PosGes src, Vect3 pos)
{
	pos[0] = src[0][3];
	pos[1] = src[1][3];
	pos[2] = src[2][3];
}
FX_VOID	FX_PGGetAxisX(PosGes src, Vect3 axis_x)
{
	axis_x[0] = src[0][0];
	axis_x[1] = src[1][0];
	axis_x[2] = src[2][0];
}
FX_VOID	FX_PGGetAxisY(PosGes src, Vect3 axis_y)
{
	axis_y[0] = src[0][1];
	axis_y[1] = src[1][1];
	axis_y[2] = src[2][1];
}
FX_VOID	FX_PGGetAxisZ(PosGes src, Vect3 axis_z)
{
	axis_z[0] = src[0][2];
	axis_z[1] = src[1][2];
	axis_z[2] = src[2][2];
}


FX_VOID FX_PGMatrixInv(Matrix4 pg, Matrix4 pginv)
{
	FX_DOUBLE p[3];
	pginv[0][0] = pg[0][0];
	pginv[0][1] = pg[1][0];
	pginv[0][2] = pg[2][0];

	pginv[1][0] = pg[0][1];
	pginv[1][1] = pg[1][1];
	pginv[1][2] = pg[2][1];

	pginv[2][0] = pg[0][2];
	pginv[2][1] = pg[1][2];
	pginv[2][2] = pg[2][2];

	pginv[3][0] = 0;
	pginv[3][1] = 0;
	pginv[3][2] = 0;
	pginv[3][3] = 1;

	p[0] = pg[0][3];
	p[1] = pg[1][3];
	p[2] = pg[2][3];

	pginv[0][3] = -(pginv[0][0] * p[0] + pginv[0][1] * p[1] + pginv[0][2] * p[2]);
	pginv[1][3] = -(pginv[1][0] * p[0] + pginv[1][1] * p[1] + pginv[1][2] * p[2]);
	pginv[2][3] = -(pginv[2][0] * p[0] + pginv[2][1] * p[1] + pginv[2][2] * p[2]);
}


FX_VOID FX_MatRotAxis(Vect3 rot_dir, FX_DOUBLE rot_angle, Matrix3 m, Matrix3 m_roted)
{
	FX_DOUBLE r[3][3];
	FX_DOUBLE sinv;
	FX_DOUBLE cosv;
	FX_DOUBLE a;

	FX_DOUBLE kx;
	FX_DOUBLE ky;
	FX_DOUBLE kz;

	FX_SIN_COS_DEG(rot_angle, &sinv, &cosv);
	a = 1 - cosv;

	kx = rot_dir[0];
	ky = rot_dir[1];
	kz = rot_dir[2];

	r[0][0] = cosv + a * kx * kx;		r[0][1] = -sinv * kz + a * kx * ky;		r[0][2] = sinv * ky + a * kx * kz;
	r[1][0] = sinv * kz + a * kx * ky;	r[1][1] = cosv + a * ky * ky;			r[1][2] = -sinv * kx + a * ky * kz;
	r[2][0] = -sinv * ky + a * kx * kz;	r[2][1] = sinv * kx + a * ky * kz;		r[2][2] = cosv + a * kz * kz;
	FX_MMM33(r, m, m_roted);
}

FX_BOOL	FX_Matrix2ZYX_DGR(Matrix3 m, FX_DOUBLE dgr, Vect3  ret)
{
	FX_DOUBLE cb;
	ret[1] = FX_ATan2(-m[2][0], FX_Sqrt(m[0][0] * m[0][0] + m[1][0] * m[1][0])) * FXARM_R2D;
	cb = FX_COS_DEG(ret[1]);
	if (FX_Fabs(cb) > dgr * FXARM_D2R)
	{
		ret[0] = FX_ATan2(m[1][0] / cb, m[0][0] / cb) * FXARM_R2D;
		ret[2] = FX_ATan2(m[2][1] / cb, m[2][2] / cb) * FXARM_R2D;
		return FX_TRUE;
	}
	ret[0] = 0;
	ret[2] = 0;
	return FX_FALSE;

}
FX_BOOL FX_Matrix2ZYX(Matrix3 m, Vect3  ret)
{
	FX_DOUBLE cb;
	ret[1] = FX_ATan2(-m[2][0], FX_Sqrt(m[0][0] * m[0][0] + m[1][0] * m[1][0])) * FXARM_R2D;
	cb = FX_COS_DEG(ret[1]);
	if (FX_Fabs(cb) > 0.05 * FXARM_D2R)
	{
		ret[0] = FX_ATan2(m[1][0] / cb, m[0][0] / cb) * FXARM_R2D;
		ret[2] = FX_ATan2(m[2][1] / cb, m[2][2] / cb) * FXARM_R2D;
		return FX_TRUE;
	}
	ret[0] = 0;
	ret[2] = 0;
	return FX_FALSE;

}

FX_VOID FX_ZYZ2Matrix(Vect3 ret, Matrix3 m)
{
	//Z(-Y)Z
	Vect3 ZYZ = { 0 };
	ZYZ[0] = ret[0];
	ZYZ[1] = -ret[1];
	ZYZ[2] = ret[2];

	FX_DOUBLE sa, ca;
	FX_DOUBLE sb, cb;
	FX_DOUBLE sr, cr;
	FX_SIN_COS_DEG(ZYZ[0], &sa, &ca);
	FX_SIN_COS_DEG(ZYZ[1], &sb, &cb);
	FX_SIN_COS_DEG(ZYZ[2], &sr, &cr);

	m[0][0] = ca * cb * cr - sa * sr;
	m[0][1] = -ca * cb * sr - sa * cr;
	m[0][2] = ca * sb;

	m[1][0] = sa * cb * cr + ca * sr;
	m[1][1] = -sa * cb * sr + ca * cr;
	m[1][2] = sa * sb;

	m[2][0] = -sb * cr;
	m[2][1] = sb * sr;
	m[2][2] = cb;

}

FX_VOID FX_ZYX2Matrix(Vect3 ret, Matrix3 m)
{
	//Z(-Y)X
	Vect3 ZYX = { 0 };
	ZYX[0] = ret[0];
	ZYX[1] = -ret[1];
	ZYX[2] = ret[2];

	FX_DOUBLE sa, ca;
	FX_DOUBLE sb, cb;
	FX_DOUBLE sr, cr;
	FX_SIN_COS_DEG(ZYX[0], &sa, &ca);
	FX_SIN_COS_DEG(ZYX[1], &sb, &cb);
	FX_SIN_COS_DEG(ZYX[2], &sr, &cr);

	m[0][0] = ca * cb;
	m[0][1] = ca * sb * sr - sa * cr;
	m[0][2] = ca * sb * cr + sa * sr;

	m[1][0] = sa * cb;
	m[1][1] = -sa * sb * sr + ca * cr;
	m[1][2] = -sa * sb * cr - ca * sr;

	m[2][0] = -sb;
	m[2][1] = cb * sr;
	m[2][2] = cb * cr;

}

FX_VOID FX_XYZ2Matrix(Vect3 ret, Matrix3 m)
{
	//ZYX
	Vect3 ZYX = { 0 };
	ZYX[0] = ret[2];
	ZYX[1] = ret[1];
	ZYX[2] = ret[0];

	FX_DOUBLE sa, ca;
	FX_DOUBLE sb, cb;
	FX_DOUBLE sr, cr;
	FX_SIN_COS_DEG(ZYX[0], &sa, &ca);
	FX_SIN_COS_DEG(ZYX[1], &sb, &cb);
	FX_SIN_COS_DEG(ZYX[2], &sr, &cr);

	m[0][0] = ca * cb;
	m[0][1] = ca * sb * sr - sa * cr;
	m[0][2] = ca * sb * cr + sa * sr;

	m[1][0] = sa * cb;
	m[1][1] = -sa * sb * sr + ca * cr;
	m[1][2] = -sa * sb * cr - ca * sr;

	m[2][0] = -sb;
	m[2][1] = cb * sr;
	m[2][2] = cb * cr;
}

FX_VOID FX_RotXYZ(Vect3 ret, Matrix3 m)
{
	//XYZ
	Vect3 XYZ = { 0 };
	XYZ[0] = ret[0];
	XYZ[1] = ret[1];
	XYZ[2] = ret[2];

	FX_DOUBLE sa, ca;
	FX_DOUBLE sb, cb;
	FX_DOUBLE sr, cr;
	FX_SIN_COS_DEG(XYZ[0], &sa, &ca);
	FX_SIN_COS_DEG(XYZ[1], &sb, &cb);
	FX_SIN_COS_DEG(XYZ[2], &sr, &cr);

	m[0][0] = cb * cr;
	m[0][1] = -cb * sr;
	m[0][2] = sb;

	m[1][0] = sa * sb * cr + ca * sr;
	m[1][1] = -sa * sb * sr + ca * cr;
	m[1][2] = -sa * cb;

	m[2][0] = -ca * sb * cr + sa * sr;
	m[2][1] = ca * sb * sr + sa * cr;
	m[2][2] = ca * cb;

}

FX_BOOL	FX_Matrix2ZYZ_DGR(Matrix3 m, FX_DOUBLE dgr, Vect3  ret)
{
	FX_DOUBLE bsin;
	FX_DOUBLE DGR;
	DGR = dgr;
	if (DGR < 0.05)
	{
		DGR = 0.05;
	}

	if (DGR > 10)
	{
		DGR = 10;
	}
	ret[1] = FX_ATan2(FX_Sqrt(m[2][0] * m[2][0] + m[2][1] * m[2][1]), m[2][2]);
	bsin = FX_SIN_ARC(ret[1]);
	if (FX_Fabs(ret[1]) < DGR * FXARM_D2R)
	{
		ret[0] = 0;
		ret[1] *= FXARM_R2D;
		ret[2] = FX_ATan2(-m[0][1], m[0][0]) * FXARM_R2D;
		return FX_FALSE;
	}
	ret[0] = FX_ATan2(m[1][2] / bsin, m[0][2] / bsin) * FXARM_R2D;
	ret[1] *= FXARM_R2D;
	ret[2] = FX_ATan2(m[2][1] / bsin, -m[2][0] / bsin) * FXARM_R2D;
	return FX_TRUE;
}
FX_BOOL FX_Matrix2ZYZ(Matrix3 m, Vect3  ret)
{
	FX_DOUBLE bsin;
	ret[1] = FX_ATan2(FX_Sqrt(m[2][0] * m[2][0] + m[2][1] * m[2][1]), m[2][2]);
	bsin = FX_SIN_ARC(ret[1]);
	if (FX_Fabs(ret[1]) < 0.05 * FXARM_D2R)
	{
		ret[0] = 0;
		ret[1] *= FXARM_R2D;
		ret[2] = FX_ATan2(-m[0][1], m[0][0]) * FXARM_R2D;
		return FX_FALSE;
	}
	ret[0] = FX_ATan2(m[1][2] / bsin, m[0][2] / bsin) * FXARM_R2D;
	ret[1] *= FXARM_R2D;
	ret[2] = FX_ATan2(m[2][1] / bsin, -m[2][0] / bsin) * FXARM_R2D;
	return FX_TRUE;
}

FX_VOID  FX_MMV3(Matrix3 L, Vect3 R, Vect3 Result)
{
	FX_INT32L i, j, k;
	for (i = 0; i < 3; i++)
	{
		Result[i] = 0;
		for (k = 0; k < 3; k++)
		{
			Result[i] += L[i][k] * R[k];
		}
	}
}


FX_VOID Tmat(FX_DOUBLE DH[4], Matrix4 T) 
{
	
	FX_DOUBLE c3;
	FX_DOUBLE s3;
	FX_DOUBLE c0;
	FX_DOUBLE s0;
	c3 = FX_COS_DEG(DH[3]);
	s3 = FX_SIN_DEG(DH[3]);
	c0 = FX_COS_DEG(DH[0]);
	s0 = FX_SIN_DEG(DH[0]);

	T[0][0] = c3; T[0][1] = -s3; T[0][2] = 0.0; T[0][3] = DH[1];
	T[1][0] = s3 * c0; T[1][1] = c3 * c0; T[1][2] = -s0; T[1][3] = -s0 * DH[2];
	T[2][0] = s3 * s0; T[2][1] = c3 * s0; T[2][2] = c0; T[2][3] = c0 * DH[2];
	T[3][0] = 0.0; T[3][1] = 0.0; T[3][2] = 0.0; T[3][3] = 1.0;
}



FX_VOID FX_PGTranXYZABC2PG(FX_DOUBLE xyzabc[6], FX_DOUBLE pg[4][4])
{
	double angx = xyzabc[3];
	double angy = xyzabc[4];
	double angz = xyzabc[5];
	double sa = 0;
	double sb = 0;
	double sr = 0;
	double ca = 0;
	double cb = 0;
	double cr = 0;
	FX_SIN_COS_DEG(angx, &sr, &cr);
	FX_SIN_COS_DEG(angy, &sb, &cb);
	FX_SIN_COS_DEG(angz, &sa, &ca);
	pg[0][0] = ca * cb;
	pg[0][1] = ca * sb * sr - sa * cr;
	pg[0][2] = ca * sb * cr + sa * sr;

	pg[1][0] = sa * cb;
	pg[1][1] = sa * sb * sr + ca * cr;
	pg[1][2] = sa * sb * cr - ca * sr;

	pg[2][0] = -sb;
	pg[2][1] = cb * sr;
	pg[2][2] = cb * cr;

	pg[0][3] = xyzabc[0];
	pg[1][3] = xyzabc[1];
	pg[2][3] = xyzabc[2];


	pg[3][0] = 0;
	pg[3][1] = 0;
	pg[3][2] = 0;
	pg[3][3] = 1;
}


//////////////////////////////////////////////////
FX_BOOL  FX_RightPsoInv67(FX_DOUBLE m[6][7], FX_DOUBLE invm[7][6])
{
	FX_INT32 i, j, k;
	FX_DOUBLE m_t[7][6];
	FX_DOUBLE FX_MMT[6][6];
	FX_DOUBLE FX_MMT_INV[6][6];

	for (i = 0; i < 6; i++)
	{
		for (j = 0; j < 7; j++)
		{
			m_t[j][i] = m[i][j];
		}
	}
	for (i = 0; i < 6; i++)
	{
		for (j = 0; j < 6; j++)
		{
			FX_MMT[i][j] = 0;
			for (k = 0; k < 7; k++)
			{
				FX_MMT[i][j] += m[i][k] * m_t[k][j];
			}
		}
	}

	if (MatrixInv66(FX_MMT, FX_MMT_INV) == FX_FALSE)
	{
		return FX_FALSE;
	}

	for (i = 0; i < 7; i++)
	{
		for (j = 0; j < 6; j++)
		{
			invm[i][j] = 0;
			for (k = 0; k < 6; k++)
			{
				invm[i][j] += m_t[i][k] * FX_MMT_INV[k][j];
			}
		}
	}

	return FX_TRUE;
}

FX_VOID PGErr(Matrix4 Td, Matrix4 Te, Vect6 err)
{
	FX_INT32 i;
	Vect3 ne, se, ae;
	Vect3 nd, sd, ad;
	Vect3 temp1, temp2, temp3;

	err[0] = Td[0][3] - Te[0][3];
	err[1] = Td[1][3] - Te[1][3];
	err[2] = Td[2][3] - Te[2][3];
	nd[0] = Td[0][0]; nd[1] = Td[1][0]; nd[2] = Td[2][0];
	sd[0] = Td[0][1]; sd[1] = Td[1][1]; sd[2] = Td[2][1];
	ad[0] = Td[0][2]; ad[1] = Td[1][2]; ad[2] = Td[2][2];
	ne[0] = Te[0][0]; ne[1] = Te[1][0]; ne[2] = Te[2][0];
	se[0] = Te[0][1]; se[1] = Te[1][1]; se[2] = Te[2][1];
	ae[0] = Te[0][2]; ae[1] = Te[1][2]; ae[2] = Te[2][2];
	FX_VectCross(ne, nd, temp1);
	FX_VectCross(se, sd, temp2);
	FX_VectCross(ae, ad, temp3);
	for (i = 0; i < 3; i++) {
		err[i + 3] = 0.5 * (temp1[i] + temp2[i] + temp3[i]);
		if (err[i + 3] > 0.1) {
			err[i + 3] = 0.1;
		}
		if (err[i + 3] < -0.1) {
			err[i + 3] = -0.1;
		}
	}
}

int Cholesky(double A[6][6], double L[6][6], int n)
{
	int i, j, k;
	for (k = 0; k < n; k++) {
		double sum = 0;
		for (i = 0; i < k; i++) {
			sum += L[k][i] * L[k][i];
		}
		sum = A[k][k] - sum;
		if (sum <= 0.0) {
			return -1;
		}
		L[k][k] = FX_Sqrt(sum);
		//L[k][k] = FX_Sqrt(sum > 0 ? sum : 0);
		for (i = k + 1; i < n; i++) {
			sum = 0;
			for (j = 0; j < k; j++) {
				sum += L[i][j] * L[k][j];
			}
			L[i][k] = (A[i][k] - sum) / L[k][k];
		}
		for (j = 0; j < k; j++) {
			L[j][k] = 0;
		}
	}
	return 0;
}

void eig(Matrix6 a, Matrix6 v, double eps)
{
	int n = 6;
	int i, j, p, q, u, w, t, s;
	double ff, fm, cn, sn, omega, x, y, d;
	for (i = 0; i <= n - 1; i++)
	{
		v[i][i] = 1.0;
		for (j = 0; j <= n - 1; j++)
			if (i != j) v[i][j] = 0.0;
	}
	ff = 0.0;
	for (i = 1; i <= n - 1; i++)
		for (j = 0; j <= i - 1; j++)
		{
			d = a[i][j]; ff = ff + d * d;
		}
	if (ff < 0.0) {

		return;
	}
	ff = FX_Sqrt(2.0 * ff);
loop0:
	ff = ff / (1.0 * n);
loop1:
	for (i = 1; i <= n - 1; i++)
		for (j = 0; j <= i - 1; j++)
		{
			d = FX_Sqrt(a[i][j]);
			if (d > ff)
			{
				p = i; q = j;
				goto loop;
			}
		}
	if (ff < eps) return;
	goto loop0;
loop: u = p * n + q; w = p * n + p; t = q * n + p; s = q * n + q;
	x = -a[p][q]; y = (a[q][q] - a[p][p]) / 2.0;
	omega = x / FX_Sqrt(x * x + y * y);
	if (y < 0.0) omega = -omega;
	sn = 1.0 + FX_Sqrt(1.0 - omega * omega);
	sn = omega / FX_Sqrt(2.0 * sn);
	cn = FX_Sqrt(1.0 - sn * sn);
	fm = a[p][p];
	a[p][p] = fm * cn * cn + a[q][q] * sn * sn + a[p][q] * omega;
	a[q][q] = fm * sn * sn + a[q][q] * cn * cn - a[p][q] * omega;
	a[p][q] = 0.0; a[q][p] = 0.0;
	for (j = 0; j <= n - 1; j++)
		if ((j != p) && (j != q))
		{
			u = p * n + j; w = q * n + j;
			fm = a[p][j];
			a[p][j] = fm * cn + a[q][j] * sn;
			a[q][j] = -fm * sn + a[q][j] * cn;
		}
	for (i = 0; i <= n - 1; i++)
		if ((i != p) && (i != q))
		{
			u = i * n + p; w = i * n + q;
			fm = a[i][p];
			a[i][p] = fm * cn + a[i][q] * sn;
			a[i][q] = -fm * sn + a[i][q] * cn;
		}
	for (i = 0; i <= n - 1; i++)
	{
		u = i * n + p; w = i * n + q;
		fm = v[i][p];
		v[i][p] = fm * cn + v[i][q] * sn;
		v[i][q] = -fm * sn + v[i][q] * cn;
	}
	goto loop1;
}

FX_BOOL generalized_eig(double eps, double A[6][6], double B[6][6], double V[6][6], double D[6])
{
	FX_INT32 i, j, tret;
	Matrix6 G = { 0 };
	Matrix6 GInv = { 0 };
	Matrix6 GInvT = { 0 };
	Matrix6 S = { 0 };
	Matrix6 temp = { 0 };
	Matrix6 y = { 0 };
	if (Cholesky(B, G, 6) != 0)
	{
		int a = 0;
	}
	tret = MatrixInv66(G, GInv);

	if (tret == 0) {
		return FX_FALSE;
	}

	FX_M66Trans(GInv, GInvT);
	FX_MMM66(A, GInvT, temp);
	FX_MMM66(GInv, temp, S);
	eig(S, y, eps);
	////*/
	D[0] = S[2][2];
	D[1] = S[1][1];
	D[2] = S[0][0];
	D[3] = S[3][3];
	D[4] = S[5][5];
	D[5] = S[4][4];


	FX_MMM66(GInvT, y, temp);
	for (i = 0; i < 6; i++) {
		V[i][0] = temp[i][2];
		V[i][1] = -temp[i][1];
		V[i][2] = -temp[i][0];
		V[i][3] = temp[i][3];
		V[i][4] = -temp[i][5];
		V[i][5] = temp[i][4];
	}

	return FX_TRUE;
}

////////////////////////////
FX_VOID	FX_Vect3AToB(Vect3 A, Vect3 B)
{
	B[0] = A[0];
	B[1] = A[1];
	B[2] = A[2];
}

FX_VOID	FX_VectAddToA(Vect3 a, Vect3 b)
{
	a[0] += b[0];
	a[1] += b[1];
	a[2] += b[2];
}

FX_VOID	FX_VectAdd(Vect3 a, Vect3 b, Vect3 result)
{
	result[0] = a[0] + b[0];
	result[1] = a[1] + b[1];
	result[2] = a[2] + b[2];
}

/////////////////////////////////////////////////////////////Quaternion///////////////
FX_VOID	FX_QuatMult(Quaternion q1, Quaternion q2, Quaternion q)
{
	FX_DOUBLE aw, ax, ay, az;
	FX_DOUBLE bw, bx, by, bz;
	aw = q1[3];
	ax = q1[0];
	ay = q1[1];
	az = q1[2];

	bw = q2[3];
	bx = q2[0];
	by = q2[1];
	bz = q2[2];
	q[0] = aw * bx + ax * bw + ay * bz - az * by;
	q[1] = aw * by + ay * bw + az * bx - ax * bz;
	q[2] = aw * bz + az * bw + ax * by - ay * bx;

	q[3] = aw * bw - ax * bx - ay * by - az * bz;
}

FX_BOOL FX_QuaternionNorm(Quaternion Q)
{
	FX_DOUBLE qq = FX_Sqrt(Q[0] * Q[0] + Q[1] * Q[1] + Q[2] * Q[2] + Q[3] * Q[3]);
	if (qq <= FXARM_EPS)
	{
		return FX_FALSE;
	}
	Q[0] /= qq;
	Q[1] /= qq;
	Q[2] /= qq;
	Q[3] /= qq;

	return FX_TRUE;
}

FX_VOID FX_ABC2Quaternions(FX_DOUBLE XYZABC[6], Quaternion Q)
{
	Matrix3 Trm;
	FX_DOUBLE sa;
	FX_DOUBLE sb;
	FX_DOUBLE sr;
	FX_DOUBLE ca;
	FX_DOUBLE cb;
	FX_DOUBLE cr;

	FX_SIN_COS_DEG(XYZABC[5], &sa, &ca);
	FX_SIN_COS_DEG(XYZABC[4], &sb, &cb);
	FX_SIN_COS_DEG(XYZABC[3], &sr, &cr);

	Trm[0][0] = ca * cb;
	Trm[0][1] = ca * sb * sr - sa * cr;
	Trm[0][2] = ca * sb * cr + sa * sr;

	Trm[1][0] = sa * cb;
	Trm[1][1] = sa * sb * sr + ca * cr;
	Trm[1][2] = sa * sb * cr - ca * sr;

	Trm[2][0] = -sb;
	Trm[2][1] = cb * sr;
	Trm[2][2] = cb * cr;


	FX_DOUBLE tr = Trm[0][0] + Trm[1][1] + Trm[2][2];
	Quaternion q;

	if (tr > 0) {
		FX_DOUBLE S = FX_Sqrt(tr + 1.0) * 2; //
		q[3] = 0.25 * S;
		q[0] = (Trm[2][1] - Trm[1][2]) / S;
		q[1] = (Trm[0][2] - Trm[2][0]) / S;
		q[2] = (Trm[1][0] - Trm[0][1]) / S;
	}
	else if ((Trm[0][0] > Trm[1][1]) && (Trm[0][0] > Trm[2][2])) {
		FX_DOUBLE S = FX_Sqrt(1.0 + Trm[0][0] - Trm[1][1] - Trm[2][2]) * 2;
		q[3] = (Trm[2][1] - Trm[1][2]) / S;
		q[0] = 0.25 * S;
		q[1] = (Trm[0][1] + Trm[1][0]) / S;
		q[2] = (Trm[0][2] + Trm[2][0]) / S;
	}
	else if (Trm[1][1] > Trm[2][2]) {
		FX_DOUBLE S = FX_Sqrt(1.0 + Trm[1][1] - Trm[0][0] - Trm[2][2]) * 2;
		q[3] = (Trm[0][2] - Trm[2][0]) / S;
		q[0] = (Trm[0][1] + Trm[1][0]) / S;
		q[1] = 0.25 * S;
		q[2] = (Trm[1][2] + Trm[2][1]) / S;
	}
	else {
		FX_DOUBLE S = FX_Sqrt(1.0 + Trm[2][2] - Trm[0][0] - Trm[1][1]) * 2;
		q[3] = (Trm[1][0] - Trm[0][1]) / S;
		q[0] = (Trm[0][2] + Trm[2][0]) / S;
		q[1] = (Trm[1][2] + Trm[2][1]) / S;
		q[2] = 0.25 * S;
	}
	FX_QuaternionNorm(q);
	Q[0] = q[0];
	Q[1] = q[1];
	Q[2] = q[2];
	Q[3] = q[3];
}


FX_VOID  FX_ABC2Q(Vect3 abc, Vect4 retq)
{
	FX_DOUBLE angx = abc[0];
	FX_DOUBLE angy = abc[1];
	FX_DOUBLE angz = abc[2];
	FX_DOUBLE sa = 0;
	FX_DOUBLE sb = 0;
	FX_DOUBLE sr = 0;
	FX_DOUBLE ca = 0;
	FX_DOUBLE cb = 0;
	FX_DOUBLE cr = 0;
	Matrix3 pg;
	FX_SIN_COS_DEG(angx, &sr, &cr);
	FX_SIN_COS_DEG(angy, &sb, &cb);
	FX_SIN_COS_DEG(angz, &sa, &ca);
	pg[0][0] = ca * cb;
	pg[0][1] = ca * sb * sr - sa * cr;
	pg[0][2] = ca * sb * cr + sa * sr;

	pg[1][0] = sa * cb;
	pg[1][1] = sa * sb * sr + ca * cr;
	pg[1][2] = sa * sb * cr - ca * sr;

	pg[2][0] = -sb;
	pg[2][1] = cb * sr;
	pg[2][2] = cb * cr;

	FX_Matrix2Quaternion3(pg, retq);

}

FX_DOUBLE FX_QuaternionSqrtNorm(Quaternion q)
{
	FX_DOUBLE qq = FX_Sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
	return qq;
}

FX_VOID FX_QuaternionConj(Quaternion q, Quaternion retq)
{
	retq[0] = -q[0];
	retq[1] = -q[1];
	retq[2] = -q[2];
	retq[3] = q[3];
}

FX_BOOL FX_QuaternionInverse(Quaternion q, Quaternion retqInv)
{
	FX_DOUBLE n2 = FX_QuaternionSqrtNorm(q);
	if (n2 > FXARM_TINYV)
	{
		retqInv[0] = -q[0] / n2;
		retqInv[1] = -q[1] / n2;
		retqInv[2] = -q[2] / n2;
		retqInv[3] = q[3] / n2;
		return FX_TRUE;
	}
	else
	{
		retqInv[0] = 1;
		retqInv[1] = 0;
		retqInv[2] = 0;
		retqInv[3] = 0;
		return FX_FALSE;
	}

}

FX_VOID FX_QuaternionSlerp(Quaternion Q_from, Quaternion Q_to, FX_DOUBLE ratio, Quaternion Q_ret)
{
	FX_DOUBLE omega, cosom, sinom, scale0, scale1;
	cosom = Q_from[0] * Q_to[0] + Q_from[1] * Q_to[1] + Q_from[2] * Q_to[2] + Q_from[3] * Q_to[3];

	if (cosom < 0.0)
	{
		cosom = -cosom;
		Q_to[0] = -Q_to[0];
		Q_to[1] = -Q_to[1];
		Q_to[2] = -Q_to[2];
		Q_to[3] = -Q_to[3];
	}

	const FX_DOUBLE EPS = 1e-6;
	if (cosom > 1.0 - EPS)
	{
		scale0 = 1.0 - ratio;
		scale1 = ratio;
	}
	else
	{
		omega = FX_ACOS(cosom);
		sinom = FX_SIN_ARC(omega);
		scale0 = FX_SIN_ARC((1.0 - ratio) * omega) / sinom;
		scale1 = FX_SIN_ARC(ratio * omega) / sinom;
	}

	Q_ret[0] = scale0 * Q_from[0] + scale1 * Q_to[0];
	Q_ret[1] = scale0 * Q_from[1] + scale1 * Q_to[1];
	Q_ret[2] = scale0 * Q_from[2] + scale1 * Q_to[2];
	Q_ret[3] = scale0 * Q_from[3] + scale1 * Q_to[3];
}

FX_VOID FX_Quaternions2ABCMatrix(Quaternion q, FX_DOUBLE xyz[3], Matrix4 m)
{
	FX_DOUBLE d11, d12, d13, d14, d22, d23, d24, d33, d34;
	d11 = q[0] * q[0];
	d12 = q[0] * q[1];
	d13 = q[0] * q[2];
	d14 = q[0] * q[3];
	d22 = q[1] * q[1];
	d23 = q[1] * q[2];
	d24 = q[1] * q[3];
	d33 = q[2] * q[2];
	d34 = q[2] * q[3];

	m[0][0] = 1 - 2 * d22 - 2 * d33;
	m[0][1] = 2 * (d12 - d34);
	m[0][2] = 2 * (d13 + d24);
	m[0][3] = xyz[0];

	m[1][0] = 2 * (d12 + d34);
	m[1][1] = 1 - 2 * d11 - 2 * d33;
	m[1][2] = 2 * (d23 - d14);
	m[1][3] = xyz[1];

	m[2][0] = 2 * (d13 - d24);
	m[2][1] = 2 * (d23 + d14);
	m[2][2] = 1 - 2 * d11 - 2 * d22;
	m[2][3] = xyz[2];

	m[3][0] = 0;
	m[3][1] = 0;
	m[3][2] = 0;
	m[3][3] = 1;
}



FX_VOID FX_Quaternions2Matrix3(Quaternion q, Matrix3 m)
{
	FX_DOUBLE d11, d12, d13, d14, d22, d23, d24, d33, d34;
	d11 = q[0] * q[0];
	d12 = q[0] * q[1];
	d13 = q[0] * q[2];
	d14 = q[0] * q[3];
	d22 = q[1] * q[1];
	d23 = q[1] * q[2];
	d24 = q[1] * q[3];
	d33 = q[2] * q[2];
	d34 = q[2] * q[3];

	m[0][0] = 1 - 2 * d22 - 2 * d33;
	m[0][1] = 2 * (d12 - d34);
	m[0][2] = 2 * (d13 + d24);


	m[1][0] = 2 * (d12 + d34);
	m[1][1] = 1 - 2 * d11 - 2 * d33;
	m[1][2] = 2 * (d23 - d14);

	m[2][0] = 2 * (d13 - d24);
	m[2][1] = 2 * (d23 + d14);
	m[2][2] = 1 - 2 * d11 - 2 * d22;

}

FX_VOID	 FX_Matrix2Quaternion3(Matrix3 m, Quaternion q)
{
	FX_DOUBLE tr = m[0][0] + m[1][1] + m[2][2];
	FX_DOUBLE S;

	if (tr > 0) {
		S = FX_Sqrt(tr + 1.0) * 2; // S=4*qw 
		q[3] = 0.25 * S;
		q[0] = (m[2][1] - m[1][2]) / S;
		q[1] = (m[0][2] - m[2][0]) / S;
		q[2] = (m[1][0] - m[0][1]) / S;
	}
	else if ((m[0][0] > m[1][1]) && (m[0][0] > m[2][2])) {
		S = FX_Sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2; // S=4*qx 
		q[3] = (m[2][1] - m[1][2]) / S;
		q[0] = 0.25 * S;
		q[1] = (m[0][1] + m[1][0]) / S;
		q[2] = (m[0][2] + m[2][0]) / S;
	}
	else if (m[1][1] > m[2][2]) {
		S = FX_Sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2; // S=4*qy
		q[3] = (m[0][2] - m[2][0]) / S;
		q[0] = (m[0][1] + m[1][0]) / S;
		q[1] = 0.25 * S;
		q[2] = (m[1][2] + m[2][1]) / S;
	}
	else {
		S = FX_Sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2; // S=4*qz
		q[3] = (m[1][0] - m[0][1]) / S;
		q[0] = (m[0][2] + m[2][0]) / S;
		q[1] = (m[1][2] + m[2][1]) / S;
		q[2] = 0.25 * S;
	}
}

FX_VOID	 FX_Matrix2Quaternion4(Matrix4 m, Quaternion q)
{
	FX_DOUBLE tr = m[0][0] + m[1][1] + m[2][2];
	FX_DOUBLE S;

	if (tr > 0) {
		S = FX_Sqrt(tr + 1.0) * 2; // S=4*qw 
		q[3] = 0.25 * S;
		q[0] = (m[2][1] - m[1][2]) / S;
		q[1] = (m[0][2] - m[2][0]) / S;
		q[2] = (m[1][0] - m[0][1]) / S;
	}
	else if ((m[0][0] > m[1][1]) && (m[0][0] > m[2][2])) {
		S = FX_Sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2; // S=4*qx 
		q[3] = (m[2][1] - m[1][2]) / S;
		q[0] = 0.25 * S;
		q[1] = (m[0][1] + m[1][0]) / S;
		q[2] = (m[0][2] + m[2][0]) / S;
	}
	else if (m[1][1] > m[2][2]) {
		S = FX_Sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2; // S=4*qy
		q[3] = (m[0][2] - m[2][0]) / S;
		q[0] = (m[0][1] + m[1][0]) / S;
		q[1] = 0.25 * S;
		q[2] = (m[1][2] + m[2][1]) / S;
	}
	else {
		S = FX_Sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2; // S=4*qz
		q[3] = (m[1][0] - m[0][1]) / S;
		q[0] = (m[0][2] + m[2][0]) / S;
		q[1] = (m[1][2] + m[2][1]) / S;
		q[2] = 0.25 * S;
	}
}

//////////////////Rotation matrix of Euler angular coordinate system
FX_VOID  FX_RotEuler_XYZ(FX_DOUBLE x_dgr, FX_DOUBLE y_dgr, FX_DOUBLE z_dgr, Matrix4 m)
{
	FX_DOUBLE sa, ca;
	FX_DOUBLE sb, cb;
	FX_DOUBLE sr, cr;
	FX_SIN_COS_DEG(x_dgr, &sa, &ca);
	FX_SIN_COS_DEG(y_dgr, &sb, &cb);
	FX_SIN_COS_DEG(z_dgr, &sr, &cr);

	m[0][0] = cb * cr;                 m[0][1] = -cb * sr;                m[0][2] = sb;       m[0][3] = 0;
	m[1][0] = sa * sb * cr + ca * sr;  m[1][1] = -sa * sb * sr + ca * cr; m[1][2] = -sa * cb; m[1][3] = 0;
	m[2][0] = -ca * sb * cr + sa * sr; m[2][1] = ca * sb * sr + sa * cr;  m[2][2] = ca * cb;  m[2][3] = 0;
	m[3][0] = 0;                       m[3][1] = 0;                       m[3][2] = 0;        m[3][3] = 1;
}

FX_VOID  FX_RotEuler_XZY(FX_DOUBLE x_dgr, FX_DOUBLE z_dgr, FX_DOUBLE y_dgr, Matrix4 m)
{
	FX_DOUBLE sa, ca;
	FX_DOUBLE sb, cb;
	FX_DOUBLE sr, cr;
	FX_SIN_COS_DEG(x_dgr, &sa, &ca);
	FX_SIN_COS_DEG(z_dgr, &sb, &cb);
	FX_SIN_COS_DEG(y_dgr, &sr, &cr);

	m[0][0] = cb * cr;                m[0][1] = -sb;     m[0][2] = cb * sr;                m[0][3] = 0;
	m[1][0] = ca * sb * cr + sa * sr; m[1][1] = ca * cb; m[1][2] = ca * sb * sr - sa * cr; m[1][3] = 0;
	m[2][0] = sa * sb * cr - ca * sr; m[2][1] = sa * cb; m[2][2] = sa * sb * sr + ca * cr; m[2][3] = 0;
	m[3][0] = 0;                      m[3][1] = 0;       m[3][2] = 0;                      m[3][3] = 1;
}

FX_VOID  FX_RotEuler_YXZ(FX_DOUBLE y_dgr, FX_DOUBLE x_dgr, FX_DOUBLE z_dgr, Matrix4 m)
{
	FX_DOUBLE sa, ca;
	FX_DOUBLE sb, cb;
	FX_DOUBLE sr, cr;
	FX_SIN_COS_DEG(y_dgr, &sa, &ca);
	FX_SIN_COS_DEG(x_dgr, &sb, &cb);
	FX_SIN_COS_DEG(z_dgr, &sr, &cr);

	m[0][0] = sa * sb * sr + ca * cr; m[0][1] = sa * sb * cr - ca * sr; m[0][2] = sa * cb; m[0][3] = 0;
	m[1][0] = cb * sr;                m[1][1] = cb * cr;                m[1][2] = -sb;     m[1][3] = 0;
	m[2][0] = ca * sb * sr - sa * cr; m[2][1] = ca * sb * cr + sa * sr; m[2][2] = ca * cb; m[2][3] = 0;
	m[3][0] = 0;                      m[3][1] = 0;                      m[3][2] = 0;       m[3][3] = 1;
}

FX_VOID  FX_RotEuler_YZX(FX_DOUBLE y_dgr, FX_DOUBLE z_dgr, FX_DOUBLE x_dgr, Matrix4 m)
{
	FX_DOUBLE sa, ca;
	FX_DOUBLE sb, cb;
	FX_DOUBLE sr, cr;
	FX_SIN_COS_DEG(y_dgr, &sa, &ca);
	FX_SIN_COS_DEG(z_dgr, &sb, &cb);
	FX_SIN_COS_DEG(x_dgr, &sr, &cr);

	m[0][0] = ca * cb;  m[0][1] = -ca * sb * cr + sa * sr; m[0][2] = ca * sb * sr + sa * cr;  m[0][3] = 0;
	m[1][0] = sb;       m[1][1] = cb * cr;                 m[1][2] = -cb * sr;                m[1][3] = 0;
	m[2][0] = -sa * cb; m[2][1] = sa * sb * cr + ca * sr;  m[2][2] = -sa * sb * sr + ca * cr; m[2][3] = 0;
	m[3][0] = 0;        m[3][1] = 0;                       m[3][2] = 0;                       m[3][3] = 1;
}

FX_VOID  FX_RotEuler_ZXY(FX_DOUBLE z_dgr, FX_DOUBLE x_dgr, FX_DOUBLE y_dgr, Matrix4 m)
{
	FX_DOUBLE sa, ca;
	FX_DOUBLE sb, cb;
	FX_DOUBLE sr, cr;
	FX_SIN_COS_DEG(z_dgr, &sa, &ca);
	FX_SIN_COS_DEG(x_dgr, &sb, &cb);
	FX_SIN_COS_DEG(y_dgr, &sr, &cr);

	m[0][0] = -sa * sb * sr + ca * cr; m[0][1] = -sa * cb; m[0][2] = sa * sb * cr + ca * sr;  m[0][3] = 0;
	m[1][0] = ca * sb * sr + sa * cr;  m[1][1] = ca * cb;  m[1][2] = -ca * sb * cr + sa * sr; m[1][3] = 0;
	m[2][0] = -cb * sr;                m[2][1] = sb;       m[2][2] = cb * cr;                 m[2][3] = 0;
	m[3][0] = 0;                       m[3][1] = 0;        m[3][2] = 0;                       m[3][3] = 1;
}

FX_VOID  FX_RotEuler_ZYX(FX_DOUBLE z_dgr, FX_DOUBLE y_dgr, FX_DOUBLE x_dgr, Matrix4 m)
{
	FX_DOUBLE sa, ca;
	FX_DOUBLE sb, cb;
	FX_DOUBLE sr, cr;
	FX_SIN_COS_DEG(z_dgr, &sa, &ca);
	FX_SIN_COS_DEG(y_dgr, &sb, &cb);
	FX_SIN_COS_DEG(x_dgr, &sr, &cr);

	m[0][0] = ca * cb; m[0][1] = ca * sb * sr - sa * cr;  m[0][2] = ca * sb * cr + sa * sr;  m[0][3] = 0;
	m[1][0] = sa * cb; m[1][1] = sa * sb * sr + ca * cr;  m[1][2] = sa * sb * cr - ca * sr;  m[1][3] = 0;
	m[2][0] = -sb;     m[2][1] = cb * sr;                 m[2][2] = cb * cr;                 m[2][3] = 0;
	m[3][0] = 0;       m[3][1] = 0;                       m[3][2] = 0;                       m[3][3] = 1;
}

FX_VOID  FX_RotEuler_XYX(FX_DOUBLE x1_dgr, FX_DOUBLE y_dgr, FX_DOUBLE x2_dgr, Matrix4 m)
{
	FX_DOUBLE sa, ca;
	FX_DOUBLE sb, cb;
	FX_DOUBLE sr, cr;
	FX_SIN_COS_DEG(x1_dgr, &sa, &ca);
	FX_SIN_COS_DEG(y_dgr, &sb, &cb);
	FX_SIN_COS_DEG(x2_dgr, &sr, &cr);

	m[0][0] = cb;       m[0][1] = sb * sr;                 m[0][2] = sb * cr;                 m[0][3] = 0;
	m[1][0] = sa * sb;  m[1][1] = -sa * cb * sr + ca * cr; m[1][2] = -sa * cb * cr - ca * sr; m[1][3] = 0;
	m[2][0] = -ca * sb; m[2][1] = ca * cb * sr + sa * cr;  m[2][2] = ca * cb * cr - sa * sr;  m[2][3] = 0;
	m[3][0] = 0;        m[3][1] = 0;                       m[3][2] = 0;                       m[3][3] = 1;
}

FX_VOID  FX_RotEuler_XZX(FX_DOUBLE x1_dgr, FX_DOUBLE z_dgr, FX_DOUBLE x2_dgr, Matrix4 m)
{
	FX_DOUBLE sa, ca;
	FX_DOUBLE sb, cb;
	FX_DOUBLE sr, cr;
	FX_SIN_COS_DEG(x1_dgr, &sa, &ca);
	FX_SIN_COS_DEG(z_dgr, &sb, &cb);
	FX_SIN_COS_DEG(x2_dgr, &sr, &cr);

	m[0][0] = cb;      m[0][1] = -sb * cr;               m[0][2] = sb * sr;                 m[0][3] = 0;
	m[1][0] = ca * sb; m[1][1] = ca * cb * cr - sa * sr; m[1][2] = -ca * cb * sr - sa * cr; m[1][3] = 0;
	m[2][0] = sa * sb; m[2][1] = sa * cb * cr + ca * sr; m[2][2] = -sa * cb * sr + ca * cr; m[2][3] = 0;
	m[3][0] = 0;       m[3][1] = 0;                      m[3][2] = 0;                       m[3][3] = 1;
}

FX_VOID  FX_RotEuler_YXY(FX_DOUBLE y1_dgr, FX_DOUBLE x_dgr, FX_DOUBLE y2_dgr, Matrix4 m)
{
	FX_DOUBLE sa, ca;
	FX_DOUBLE sb, cb;
	FX_DOUBLE sr, cr;
	FX_SIN_COS_DEG(y1_dgr, &sa, &ca);
	FX_SIN_COS_DEG(x_dgr, &sb, &cb);
	FX_SIN_COS_DEG(y2_dgr, &sr, &cr);

	m[0][0] = -sa * cb * sr + ca * cr; m[0][1] = sa * sb; m[0][2] = sa * cb * cr + ca * sr; m[0][3] = 0;
	m[1][0] = sb * sr;                 m[1][1] = cb;      m[1][2] = -sb * cr;               m[1][3] = 0;
	m[2][0] = -ca * cb * sr - sa * cr; m[2][1] = ca * sb; m[2][2] = ca * cb * cr - sa * sr; m[2][3] = 0;
	m[3][0] = 0;                       m[3][1] = 0;       m[3][2] = 0;                      m[3][3] = 1;
}

FX_VOID  FX_RotEuler_YZY(FX_DOUBLE y1_dgr, FX_DOUBLE z_dgr, FX_DOUBLE y2_dgr, Matrix4 m)
{
	FX_DOUBLE sa, ca;
	FX_DOUBLE sb, cb;
	FX_DOUBLE sr, cr;
	FX_SIN_COS_DEG(y1_dgr, &sa, &ca);
	FX_SIN_COS_DEG(z_dgr, &sb, &cb);
	FX_SIN_COS_DEG(y2_dgr, &sr, &cr);

	m[0][0] = ca * cb * cr - sa * sr;  m[0][1] = -ca * sb; m[0][2] = ca * cb * sr + sa * cr;  m[0][3] = 0;
	m[1][0] = sb * cr;                 m[1][1] = cb;       m[1][2] = sb * sr;                 m[1][3] = 0;
	m[2][0] = -sa * cb * cr - ca * sr; m[2][1] = sa * sb;  m[2][2] = -sa * cb * sr + ca * cr; m[2][3] = 0;
	m[3][0] = 0;                       m[3][1] = 0;        m[3][2] = 0;                       m[3][3] = 1;
}

FX_VOID  FX_RotEuler_ZXZ(FX_DOUBLE z1_dgr, FX_DOUBLE x_dgr, FX_DOUBLE z2_dgr, Matrix4 m)
{
	FX_DOUBLE sa, ca;
	FX_DOUBLE sb, cb;
	FX_DOUBLE sr, cr;
	FX_SIN_COS_DEG(z1_dgr, &sa, &ca);
	FX_SIN_COS_DEG(x_dgr, &sb, &cb);
	FX_SIN_COS_DEG(z2_dgr, &sr, &cr);

	m[0][0] = -sa * cb * sr + ca * cr; m[0][1] = -sa * cb * cr - ca * sr; m[0][2] = sa * sb;  m[0][3] = 0;
	m[1][0] = ca * cb * sr + sa * cr;  m[1][1] = ca * cb * cr - sa * sr;  m[1][2] = -ca * sb; m[1][3] = 0;
	m[2][0] = sb * sr;                 m[2][1] = cr * sb;                 m[2][2] = cb;       m[2][3] = 0;
	m[3][0] = 0;                       m[3][1] = 0;                       m[3][2] = 0;        m[3][3] = 1;
}

FX_VOID  FX_RotEuler_ZYZ(FX_DOUBLE z1_dgr, FX_DOUBLE y_dgr, FX_DOUBLE z2_dgr, Matrix4 m)
{
	FX_DOUBLE sa, ca;
	FX_DOUBLE sb, cb;
	FX_DOUBLE sr, cr;
	FX_SIN_COS_DEG(z1_dgr, &sa, &ca);
	FX_SIN_COS_DEG(y_dgr, &sb, &cb);
	FX_SIN_COS_DEG(z2_dgr, &sr, &cr);

	m[0][0] = ca * cb * cr - sa * sr; m[0][1] = -ca * cb * sr - sa * cr; m[0][2] = ca * sb; m[0][3] = 0;
	m[1][0] = sa * cb * cr + ca * sr; m[1][1] = -sa * cb * sr + ca * cr; m[1][2] = sa * sb; m[1][3] = 0;
	m[2][0] = -sb * cr;               m[2][1] = sr * sb;                 m[2][2] = cb;      m[2][3] = 0;
	m[3][0] = 0;                      m[3][1] = 0;                       m[3][2] = 0;       m[3][3] = 1;
}

//////////////////Rotation matrix of fixed coordinate system
FX_VOID  FX_RotFixed_XYZ(FX_DOUBLE x_dgr, FX_DOUBLE y_dgr, FX_DOUBLE z_dgr, Matrix4 m)
{
	FX_DOUBLE sa, ca;
	FX_DOUBLE sb, cb;
	FX_DOUBLE sr, cr;
	FX_SIN_COS_DEG(z_dgr, &sa, &ca);
	FX_SIN_COS_DEG(y_dgr, &sb, &cb);
	FX_SIN_COS_DEG(x_dgr, &sr, &cr);

	m[0][0] = ca * cb; m[0][1] = ca * sb * sr - sa * cr; m[0][2] = ca * sb * cr + sa * sr; m[0][3] = 0;
	m[1][0] = sa * cb; m[1][1] = sa * sb * sr + ca * cr; m[1][2] = sa * sb * cr - ca * sr; m[1][3] = 0;
	m[2][0] = -sb;     m[2][1] = cb * sr;                m[2][2] = cb * cr;                m[2][3] = 0;
	m[3][0] = 0;       m[3][1] = 0;                      m[3][2] = 0;                      m[3][3] = 1;
}

FX_VOID  FX_RotFixed_XZY(FX_DOUBLE x_dgr, FX_DOUBLE z_dgr, FX_DOUBLE y_dgr, Matrix4 m)
{
	FX_DOUBLE sa, ca;
	FX_DOUBLE sb, cb;
	FX_DOUBLE sr, cr;
	FX_SIN_COS_DEG(y_dgr, &sa, &ca);
	FX_SIN_COS_DEG(z_dgr, &sb, &cb);
	FX_SIN_COS_DEG(x_dgr, &sr, &cr);

	m[0][0] = ca * cb;  m[0][1] = -ca * sb * cr + sa * sr; m[0][2] = ca * sb * sr + sa * cr;  m[0][3] = 0;
	m[1][0] = sb;       m[1][1] = cb * cr;                 m[1][2] = -cb * sr;                m[1][3] = 0;
	m[2][0] = -sa * cb; m[2][1] = sa * sb * cr + ca * sr;  m[2][2] = -sa * sb * sr + ca * cr; m[2][3] = 0;
	m[3][0] = 0;        m[3][1] = 0;                       m[3][2] = 0;                       m[3][3] = 1;
}

FX_VOID  FX_RotFixed_YXZ(FX_DOUBLE y_dgr, FX_DOUBLE x_dgr, FX_DOUBLE z_dgr, Matrix4 m)
{
	FX_DOUBLE sa, ca;
	FX_DOUBLE sb, cb;
	FX_DOUBLE sr, cr;
	FX_SIN_COS_DEG(z_dgr, &sa, &ca);
	FX_SIN_COS_DEG(x_dgr, &sb, &cb);
	FX_SIN_COS_DEG(y_dgr, &sr, &cr);

	m[0][0] = -sa * sb * sr + ca * cr; m[0][1] = -sa * cb; m[0][2] = sa * sb * cr + ca * sr;  m[0][3] = 0;
	m[1][0] = ca * sb * sr + sa * cr;  m[1][1] = ca * cb;  m[1][2] = -ca * sb * cr + sa * sr; m[1][3] = 0;
	m[2][0] = -cb * sr;                m[2][1] = sb;       m[2][2] = cb * cr;                 m[2][3] = 0;
	m[3][0] = 0;                       m[3][1] = 0;        m[3][2] = 0;                       m[3][3] = 1;
}

FX_VOID  FX_RotFixed_YZX(FX_DOUBLE y_dgr, FX_DOUBLE z_dgr, FX_DOUBLE x_dgr, Matrix4 m)
{
	FX_DOUBLE sa, ca;
	FX_DOUBLE sb, cb;
	FX_DOUBLE sr, cr;
	FX_SIN_COS_DEG(x_dgr, &sa, &ca);
	FX_SIN_COS_DEG(z_dgr, &sb, &cb);
	FX_SIN_COS_DEG(y_dgr, &sr, &cr);

	m[0][0] = cb * cr;                m[0][1] = -sb;     m[0][2] = cb * sr;                m[0][3] = 0;
	m[1][0] = ca * sb * cr + sa * sr; m[1][1] = ca * cb; m[1][2] = ca * sb * sr - sa * cr; m[1][3] = 0;
	m[2][0] = sa * sb * cr - ca * sr; m[2][1] = sa * cb; m[2][2] = sa * sb * sr + ca * cr; m[2][3] = 0;
	m[3][0] = 0;                      m[3][1] = 0;       m[3][2] = 0;                      m[3][3] = 1;
}

FX_VOID  FX_RotFixed_ZXY(FX_DOUBLE z_dgr, FX_DOUBLE x_dgr, FX_DOUBLE y_dgr, Matrix4 m)
{
	FX_DOUBLE sa, ca;
	FX_DOUBLE sb, cb;
	FX_DOUBLE sr, cr;
	FX_SIN_COS_DEG(y_dgr, &sa, &ca);
	FX_SIN_COS_DEG(x_dgr, &sb, &cb);
	FX_SIN_COS_DEG(z_dgr, &sr, &cr);

	m[0][0] = sa * sb * sr + ca * cr; m[0][1] = sa * sb * cr - ca * sr; m[0][2] = sa * cb; m[0][3] = 0;
	m[1][0] = cb * sr;                m[1][1] = cb * cr;                m[1][2] = -sb;     m[1][3] = 0;
	m[2][0] = ca * sb * sr - sa * cr; m[2][1] = ca * sb * cr + sa * sr; m[2][2] = ca * cb; m[2][3] = 0;
	m[3][0] = 0;                      m[3][1] = 0;                      m[3][2] = 0;       m[3][3] = 1;
}

FX_VOID  FX_RotFixed_ZYX(FX_DOUBLE z_dgr, FX_DOUBLE y_dgr, FX_DOUBLE x_dgr, Matrix4 m)
{
	FX_DOUBLE sa, ca;
	FX_DOUBLE sb, cb;
	FX_DOUBLE sr, cr;
	FX_SIN_COS_DEG(x_dgr, &sa, &ca);
	FX_SIN_COS_DEG(y_dgr, &sb, &cb);
	FX_SIN_COS_DEG(z_dgr, &sr, &cr);

	m[0][0] = cb * cr;                 m[0][1] = -cb * sr;                m[0][2] = sb;       m[0][3] = 0;
	m[1][0] = sa * sb * cr + ca * sr;  m[1][1] = -sa * sb * sr + ca * cr; m[1][2] = -sa * cb; m[1][3] = 0;
	m[2][0] = -ca * sb * cr + sa * sr; m[2][1] = ca * sb * sr + sa * cr;  m[2][2] = ca * cb;  m[2][3] = 0;
	m[3][0] = 0;                       m[3][1] = 0;                       m[3][2] = 0;        m[3][3] = 1;
}

FX_VOID  FX_RotFixed_XYX(FX_DOUBLE x1_dgr, FX_DOUBLE y_dgr, FX_DOUBLE x2_dgr, Matrix4 m)
{
	FX_DOUBLE sa, ca;
	FX_DOUBLE sb, cb;
	FX_DOUBLE sr, cr;
	FX_SIN_COS_DEG(x2_dgr, &sa, &ca);
	FX_SIN_COS_DEG(y_dgr, &sb, &cb);
	FX_SIN_COS_DEG(x1_dgr, &sr, &cr);

	m[0][0] = cb;       m[0][1] = sb * sr;                 m[0][2] = sb * cr;                 m[0][3] = 0;
	m[1][0] = sa * sb;  m[1][1] = -sa * cb * sr + ca * cr; m[1][2] = -sa * cb * cr - ca * sr; m[1][3] = 0;
	m[2][0] = -ca * sb; m[2][1] = ca * cb * sr + sa * cr;  m[2][2] = ca * cb * cr - sa * sr;  m[2][3] = 0;
	m[3][0] = 0;        m[3][1] = 0;                       m[3][2] = 0;                       m[3][3] = 1;
}

FX_VOID  FX_RotFixed_XZX(FX_DOUBLE x1_dgr, FX_DOUBLE z_dgr, FX_DOUBLE x2_dgr, Matrix4 m)
{
	FX_DOUBLE sa, ca;
	FX_DOUBLE sb, cb;
	FX_DOUBLE sr, cr;
	FX_SIN_COS_DEG(x2_dgr, &sa, &ca);
	FX_SIN_COS_DEG(z_dgr, &sb, &cb);
	FX_SIN_COS_DEG(x1_dgr, &sr, &cr);

	m[0][0] = cb;      m[0][1] = -sb * cr;               m[0][2] = sb * sr;                 m[0][3] = 0;
	m[1][0] = ca * sb; m[1][1] = ca * cb * cr - sa * sr; m[1][2] = -ca * cb * sr - sa * cr; m[1][3] = 0;
	m[2][0] = sa * sb; m[2][1] = sa * cb * cr + ca * sr; m[2][2] = -sa * cb * sr + ca * cr; m[2][3] = 0;
	m[3][0] = 0;       m[3][1] = 0;                      m[3][2] = 0;                       m[3][3] = 1;
}

FX_VOID  FX_RotFixed_YXY(FX_DOUBLE y1_dgr, FX_DOUBLE x_dgr, FX_DOUBLE y2_dgr, Matrix4 m)
{
	FX_DOUBLE sa, ca;
	FX_DOUBLE sb, cb;
	FX_DOUBLE sr, cr;
	FX_SIN_COS_DEG(y2_dgr, &sa, &ca);
	FX_SIN_COS_DEG(x_dgr, &sb, &cb);
	FX_SIN_COS_DEG(y1_dgr, &sr, &cr);

	m[0][0] = -sa * cb * sr + ca * cr; m[0][1] = sa * sb; m[0][2] = sa * cb * cr + ca * sr; m[0][3] = 0;
	m[1][0] = sb * sr;                 m[1][1] = cb;      m[1][2] = -sb * cr;               m[1][3] = 0;
	m[2][0] = -ca * cb * sr - sa * cr; m[2][1] = ca * sb; m[2][2] = ca * cb * cr - sa * sr; m[2][3] = 0;
	m[3][0] = 0;                       m[3][1] = 0;       m[3][2] = 0;                      m[3][3] = 1;
}

FX_VOID  FX_RotFixed_YZY(FX_DOUBLE y1_dgr, FX_DOUBLE z_dgr, FX_DOUBLE y2_dgr, Matrix4 m)
{
	FX_DOUBLE sa, ca;
	FX_DOUBLE sb, cb;
	FX_DOUBLE sr, cr;
	FX_SIN_COS_DEG(y2_dgr, &sa, &ca);
	FX_SIN_COS_DEG(z_dgr, &sb, &cb);
	FX_SIN_COS_DEG(y1_dgr, &sr, &cr);

	m[0][0] = ca * cb * cr - sa * sr;  m[0][1] = -ca * sb; m[0][2] = ca * cb * sr + sa * cr;  m[0][3] = 0;
	m[1][0] = sb * cr;                 m[1][1] = cb;       m[1][2] = sb * sr;                 m[1][3] = 0;
	m[2][0] = -sa * cb * cr - ca * sr; m[2][1] = sa * sb;  m[2][2] = -sa * cb * sr + ca * cr; m[2][3] = 0;
	m[3][0] = 0;                       m[3][1] = 0;        m[3][2] = 0;                       m[3][3] = 1;
}

FX_VOID  FX_RotFixed_ZXZ(FX_DOUBLE z1_dgr, FX_DOUBLE x_dgr, FX_DOUBLE z2_dgr, Matrix4 m)
{
	FX_DOUBLE sa, ca;
	FX_DOUBLE sb, cb;
	FX_DOUBLE sr, cr;
	FX_SIN_COS_DEG(z2_dgr, &sa, &ca);
	FX_SIN_COS_DEG(x_dgr, &sb, &cb);
	FX_SIN_COS_DEG(z1_dgr, &sr, &cr);

	m[0][0] = -sa * cb * sr + ca * cr; m[0][1] = -sa * cb * cr - ca * sr; m[0][2] = sa * sb;  m[0][3] = 0;
	m[1][0] = ca * cb * sr + sa * cr;  m[1][1] = ca * cb * cr - sa * sr;  m[1][2] = -ca * sb; m[1][3] = 0;
	m[2][0] = sb * sr;                 m[2][1] = cr * sb;                 m[2][2] = cb;       m[2][3] = 0;
	m[3][0] = 0;                       m[3][1] = 0;                       m[3][2] = 0;        m[3][3] = 1;
}

FX_VOID  FX_RotFixed_ZYZ(FX_DOUBLE z1_dgr, FX_DOUBLE y_dgr, FX_DOUBLE z2_dgr, Matrix4 m)
{
	FX_DOUBLE sa, ca;
	FX_DOUBLE sb, cb;
	FX_DOUBLE sr, cr;
	FX_SIN_COS_DEG(z1_dgr, &sa, &ca);
	FX_SIN_COS_DEG(y_dgr, &sb, &cb);
	FX_SIN_COS_DEG(z2_dgr, &sr, &cr);

	m[0][0] = ca * cb * cr - sa * sr; m[0][1] = -ca * cb * sr - sa * cr; m[0][2] = ca * sb; m[0][3] = 0;
	m[1][0] = sa * cb * cr + ca * sr; m[1][1] = -sa * cb * sr + ca * cr; m[1][2] = sa * sb; m[1][3] = 0;
	m[2][0] = -sb * cr;               m[2][1] = sr * sb;                 m[2][2] = cb;      m[2][3] = 0;
	m[3][0] = 0;                      m[3][1] = 0;                       m[3][2] = 0;       m[3][3] = 1;
}
