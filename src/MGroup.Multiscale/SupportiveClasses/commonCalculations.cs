using System;
using System.Collections.Generic;
using System.Text;

namespace ISAAR.MSolve.MultiscaleAnalysis.SupportiveClasses
{
    class commonCalculations
    {
        public static double[] invert3by3(double[,] A, double[] b)
        {
            double[,] A_inverse = new double[3, 3] { { A[2, 2]*A[1,1]- A[2, 1] * A[1, 2],-(A[2, 2] * A[0, 1] - A[2, 1] * A[0, 2]), A[1,2] * A[0, 1] - A[1, 1] * A[0, 2] },
                { -(A[2,2]*A[1,0]-A[2,0]*A[1,2]),A[2,2]*A[0,0]-A[2,0]*A[0,2], -(A[1,2]*A[0,0]-A[1,0]*A[0,2]) },
                {A[2,1]*A[1,0]-A[2,0]*A[1,1],-(A[2,1]*A[0,0]-A[2,0]*A[0,1]),A[1,1]*A[0,0]-A[1,0]*A[0,1] } };



            double det_A;

            det_A = A[0, 0] * (A[2, 2] * A[1, 1] - A[2, 1] * A[1, 2]) - A[1, 0] * (A[2, 2] * A[0, 1] - A[2, 1] * A[0, 2]) + A[2, 0] * (A[1, 2] * A[0, 1] - A[1, 1] * A[0, 2]);
            for (int q2 = 0; q2 < 3; q2++)
            {
                for (int q1 = 0; q1 < 3; q1++)
                { A_inverse[q1, q2] = A_inverse[q1, q2] / det_A; }
            }

            double[] x = commonCalculations.MatVecMult(A_inverse, b);
            return x;

        }

        public static double[] MatVecMult(double[,] matrix, double[] vec)
        {
            double[] product = new double[matrix.GetLength(0)];
            for (int q1 = 0; q1 < matrix.GetLength(0); q1++)
            {
                for (int q2 = 0; q2 < matrix.GetLength(1); q2++)
                {
                    product[q1] += matrix[q1, q2] * vec[q2];
                }
            }
            return product;
        }

        public static double dot_product(double[] vec1, double[] vec2)
        {
            double dot_product = 0;
            for (int i1 = 0; i1 < vec1.GetLength(0); i1++)
            {
                dot_product += vec1[i1] * vec2[i1];
            }
            return dot_product;
        }
		public static double[] invert2by2(double[,] A, double[] b)
		{
			double[,] A_inverse = new double[2, 2] { { A[1, 1], -A[0, 1]   },
				{-A[1,0], A[0,0]} };



			double det_A = A[0, 0] * A[1, 1] - A[0, 1] * A[1, 0];

			for (int q2 = 0; q2 < 2; q2++)
			{
				for (int q1 = 0; q1 < 2; q1++)
				{ A_inverse[q1, q2] = A_inverse[q1, q2] / det_A; }
			}

			double[] x = commonCalculations.MatVecMult(A_inverse, b);
			return x;

		}
	}
}
