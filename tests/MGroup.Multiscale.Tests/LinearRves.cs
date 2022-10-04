using MGroup.Constitutive.Structural.Continuum;
using MGroup.Constitutive.Structural.Planar;
using MGroup.Constitutive.Structural.Shells;
using MGroup.LinearAlgebra.Commons;
using MGroup.LinearAlgebra.Matrices;
using MGroup.LinearAlgebra.Vectors;
using MGroup.MSolve.MultiscaleAnalysis;
//using MGroup.Materials;
//using MGroup.Materials.Interfaces;
//using MGroup.Materials.ShellMaterials;
using MGroup.MSolve.MultiscaleAnalysis.Interfaces;
//using MGroup.Multiscale.Interfaces;
using MGroup.Solvers.Direct;

using MiMsolve.SolutionStrategies;

using Xunit;

namespace MGroup.Multiscale.Tests
{
	public static class LinearRves
	{
		[Fact]
		public static void CheckShellScaleTransitionsAndMicrostructure()
		{
			//Origin: Check05fStressIntegration
			//alllages: use of updated v2 classes


			double E_disp = 3.5; /*Gpa*/ double ni_disp = 0.4; // stather Poisson
															   //var material1 = new ElasticMaterial2D(StressState2D.PlaneStress)
															   //{ YoungModulus = E_disp, PoissonRatio = ni_disp, };
															   //double[] GLVec = new double[3] { 0.01, 0, 0 };
															   //material1.UpdateMaterial(new StressStrainVectorContinuum2D(GLVec));
															   //double[] stressesCheck1 = new double[3] { material1.Stresses[0], material1.Stresses[1], material1.Stresses[2] };

			var Vec1 = Vector.CreateFromArray(new double[3] { 1, 0, 0 });
			var Vec2 = Vector.CreateFromArray(new double[3] { 0.5, 2, 0 });
			var strain = new double[3] { 0.01, 0, 0 };

			//var material2 = new ShellElasticMaterial2D() { YoungModulus = E_disp, PoissonRatio = ni_disp, TangentVectorV1 = new double[3] { Vec1[0], Vec1[1], Vec1[2] }, TangentVectorV2 = new double[3] { Vec2[0], Vec2[1], Vec2[2] } };
			//material2.UpdateMaterial(strain);
			//double[] stressesCheck2 = new double[3] { material2.Stresses[0], material2.Stresses[1], material2.Stresses[2] };

			var material3 = new ShellElasticMaterial2Dtransformationb(E_disp, ni_disp);
			material3.TangentVectorV1 = new double[3] { Vec1[0], Vec1[1], Vec1[2] };
			material3.TangentVectorV2 = new double[3] { Vec2[0], Vec2[1], Vec2[2] };
			var Matrix1 = Matrix.CreateZero(3, 3); for (int i1 = 0; i1 < 3; i1++) { for (int i2 = 0; i2 < 3; i2++) { Matrix1[i1, i2] = material3.ConstitutiveMatrix[i1, i2]; } }
			material3.UpdateConstitutiveMatrixAndEvaluateResponse(strain);
			double[] stressesCheck3 = new double[3] { material3.Stresses[0], material3.Stresses[1], material3.Stresses[2] };

			//VectorExtensions.AssignTotalAffinityCount();
			IdegenerateRVEbuilder homogeneousRveBuilder1 = new HomogeneousRVEBuilderLinearAndDegenerate();
			//IRVEbuilder homogeneousRveBuilder1 = new HomogeneousRVEBuilderCheckEnaHexa();

			//IContinuumMaterial2D microstructure3 = new Microstructure3DevelopMultipleSubdomainsUseBaseSmallStrains2D(homogeneousRveBuilder1);
			//IContinuumMaterial3DDefGrad microstructure3copyConsCheck = new Microstructure3copyConsCheckEna(homogeneousRveBuilder1);
			double[,] consCheck1 = new double[3, 3];
			for (int i1 = 0; i1 < 3; i1++) { for (int i2 = 0; i2 < 3; i2++) { consCheck1[i1, i2] = material3.ConstitutiveMatrix[i1, i2]; } }

			var material4 = new MicrostructureShell2D<SkylineMatrix>(homogeneousRveBuilder1, false, 1, new SkylineSolverPrefernce())
			{
				TangentVectorV1 = new double[3] { Vec1[0], Vec1[1], Vec1[2] },
				TangentVectorV2 = new double[3] { Vec2[0], Vec2[1], Vec2[2] }
			};
			var Matrix2 = Matrix.CreateZero(3, 3); for (int i1 = 0; i1 < 3; i1++) { for (int i2 = 0; i2 < 3; i2++) { Matrix2[i1, i2] = material4.ConstitutiveMatrix[i1, i2]; } }
			material4.UpdateConstitutiveMatrixAndEvaluateResponse(strain);
			double[] stressesCheck4 = new double[3] { material4.Stresses[0], material4.Stresses[1], material4.Stresses[2] };


			//-------------Check 2 steps savestate etc---------------
			material4.CreateState();
			material4.UpdateConstitutiveMatrixAndEvaluateResponse(new double[3] { 2 * strain[0], 2 * strain[1], 2 * strain[2] });
			double[] stressesCheck5 = new double[3] { material4.Stresses[0], material4.Stresses[1], material4.Stresses[2] };

			Assert.True(AreDisplacementsSame(stressesCheck3, stressesCheck4));
			Assert.True(AreDisplacementsSame(new double[3] { 2 * stressesCheck3[0], 2 * stressesCheck3[1], 2 * stressesCheck3[2] },
																			stressesCheck5));
			Assert.True(BondSlipTest.AreDisplacementsSame(Matrix1.CopyToArray2D(), consCheck1));
			Assert.True(AreDisplacementsSame(Matrix1.CopyToArray2D(), material4.ConstitutiveMatrix));
		}

		[Fact]
		public static void Check2DscaleTransitionsAndMicrostructure()
		{
			////Check05cStressIntegration()
			//double E_disp = 3.5; /*Gpa*/ double ni_disp = 0.4; // stathera Poisson
			//var material1 = new ElasticMaterial2D(E_disp, ni_disp, StressState2D.PlaneStress);
			//double[] GLVec = new double[3] { 0.01, 0, 0 };
			//material1.UpdateConstitutiveMatrixAndEvaluateResponse(GLVec);
			//double[] stressesCheck1 = new double[3] { material1.Stresses[0], material1.Stresses[1], material1.Stresses[2] };
			////material1.SaveState();
			//GLVec = new double[3] { 0.02, 0, 0 };
			//material1.UpdateConstitutiveMatrixAndEvaluateResponse(GLVec);
			//double[] stressesCheck2 = new double[3] { material1.Stresses[0], material1.Stresses[1], material1.Stresses[2] };



			double[] stressesCheck1 = new double[] { 0.041666666666666671, 0.01666666666666667, 0 };
			double[] stressesCheck2 = new double[] { 0.083333333333333343, 0.03333333333333334, 0 };
			var consMatrix = Matrix.CreateFromArray(new double[,] { { 4.166666666666667, 1.666666666666667, 0 }, { 1.666666666666667, 4.166666666666667, 0 }, { 0, 0, 1.25 } });

			//VectorExtensions.AssignTotalAffinityCount();
			IdegenerateRVEbuilder homogeneousRveBuilder1 = new HomogeneousRVEBuilderLinearAndDegenerate();
			//IRVEbuilder homogeneousRveBuilder1 = new HomogeneousRVEBuilderCheckEnaHexa();

			IContinuumMaterial2D microstructure3 = new Microstructure2DplaneStress<SkylineMatrix>(homogeneousRveBuilder1, false, 1, new SkylineSolverPrefernce());
			//IContinuumMaterial3DDefGrad microstructure3copyConsCheck = new Microstructure3copyConsCheckEna(homogeneousRveBuilder1);
			double[,] consCheck1 = new double[3, 3];
			for (int i1 = 0; i1 < 3; i1++) { for (int i2 = 0; i2 < 3; i2++) { consCheck1[i1, i2] = microstructure3.ConstitutiveMatrix[i1, i2]; } }

			double[] stresses = microstructure3.UpdateConstitutiveMatrixAndEvaluateResponse(new double[3] { 0.010, 0, 0 });
			double[] stressesCheck3 = new double[3] { stresses[0], stresses[1], stresses[2] };
			microstructure3.CreateState();
			stresses = microstructure3.UpdateConstitutiveMatrixAndEvaluateResponse(new double[3] { 0.020, 0, 0 });
			double[] stressesCheck4 = new double[3] { stresses[0], stresses[1], stresses[2] };

			microstructure3.CreateState();
			stresses = microstructure3.UpdateConstitutiveMatrixAndEvaluateResponse(new double[3] { 0.030, 0, 0 });
			double[] stressesCheck5 = new double[3] { stresses[0], stresses[1], stresses[2] };
			var Matrix1 = Matrix.CreateZero(3, 3); for (int i1 = 0; i1 < 3; i1++) { for (int i2 = 0; i2 < 3; i2++) { Matrix1[i1, i2] = microstructure3.ConstitutiveMatrix[i1, i2]; } }

			Assert.True(AreDisplacementsSame(stressesCheck1, stressesCheck3));
			Assert.True(AreDisplacementsSame(stressesCheck2, stressesCheck4));
			Assert.True(AreDisplacementsSame(new double[3] { 3 * stressesCheck1[0], 3 * stressesCheck1[1], 3 * stressesCheck1[2] },
																			stressesCheck5));
			Assert.True(AreDisplacementsSame(consCheck1, consMatrix));
			Assert.True(AreDisplacementsSame(Matrix1.CopyToArray2D(), consMatrix));
		}

		[Fact]
		public static void Check3DscaleTransitionsAndMicrostructure()
		{
			//Check05c2_3D_StressIntegration
			double E_disp = 3.5; /*Gpa*/ double ni_disp = 0.4; // stather Poisson
			var material1 = new ElasticMaterial3D(E_disp, ni_disp);
			double[] GLVec = new double[6] { 0.01, 0, 0, 0, 0, 0 };
			material1.UpdateConstitutiveMatrixAndEvaluateResponse(GLVec);
			double[] stressesCheck1 = new double[6] { material1.Stresses[0], material1.Stresses[1], material1.Stresses[2], material1.Stresses[3], material1.Stresses[4], material1.Stresses[5] };
			//material1.SaveState();
			GLVec = new double[6] { 0, 0, 0, 0, 0.02, 0 };
			material1.UpdateConstitutiveMatrixAndEvaluateResponse(GLVec);
			double[] stressesCheck2 = new double[6] { material1.Stresses[0], material1.Stresses[1], material1.Stresses[2], material1.Stresses[3], material1.Stresses[4], material1.Stresses[5] };

			//VectorExtensions.AssignTotalAffinityCount();
			IRVEbuilder homogeneousRveBuilder1 = new HomogeneousRVEBuilderLinear();
			//IRVEbuilder homogeneousRveBuilder1 = new HomogeneousRVEBuilderCheckEnaHexa();

			IContinuumMaterial3D microstructure3 = new Microstructure3D<SkylineMatrix>(homogeneousRveBuilder1, false, 1, new SkylineSolverPrefernce());
			//IContinuumMaterial3DDefGrad microstructure3copyConsCheck = new Microstructure3copyConsCheckEna(homogeneousRveBuilder1);
			double[,] consCheck1 = new double[6, 6];
			for (int i1 = 0; i1 < 6; i1++) { for (int i2 = 0; i2 < 6; i2++) { consCheck1[i1, i2] = microstructure3.ConstitutiveMatrix[i1, i2]; } }

			double[] stresses = microstructure3.UpdateConstitutiveMatrixAndEvaluateResponse(new double[6] { 0.010, 0, 0, 0, 0, 0 });
			double[] stressesCheck3 = new double[6] { stresses[0], stresses[1], stresses[2], stresses[3], stresses[4], stresses[5] };
			microstructure3.CreateState();
			stresses = microstructure3.UpdateConstitutiveMatrixAndEvaluateResponse(new double[6] { 0, 0, 0, 0, 0.020, 0 });
			double[] stressesCheck4 = new double[6] { stresses[0], stresses[1], stresses[2], stresses[3], stresses[4], stresses[5] };

			microstructure3.CreateState();
			stresses = microstructure3.UpdateConstitutiveMatrixAndEvaluateResponse(new double[6] { 0.030, 0, 0, 0, 0, 0 });
			double[] stressesCheck5 = new double[6] { stresses[0], stresses[1], stresses[2], stresses[3], stresses[4], stresses[5] };
			var Matrix1 = Matrix.CreateZero(3, 3); for (int i1 = 0; i1 < 3; i1++) { for (int i2 = 0; i2 < 3; i2++) { Matrix1[i1, i2] = microstructure3.ConstitutiveMatrix[i1, i2]; } }

			Assert.True(AreDisplacementsSame(stressesCheck1, stressesCheck3));
			Assert.True(AreDisplacementsSame(stressesCheck2, stressesCheck4));
			Assert.True(AreDisplacementsSame(new double[6] { 3 * stressesCheck1[0], 3 * stressesCheck1[1], 3 * stressesCheck1[2], 3 * stressesCheck1[3], 3 * stressesCheck1[4], 3 * stressesCheck1[5] },
																			stressesCheck5));
			Assert.True(AreDisplacementsSame(consCheck1, material1.ConstitutiveMatrix));
			Assert.True(AreDisplacementsSame(Matrix1.CopyToArray2D(), material1.ConstitutiveMatrix));
		}

		public static bool AreDisplacementsSame(double[,] expectedValues,
			IMatrixView computedValues)
		{
			var comparer = new ValueComparer(1E-14);
			for (int i1 = 0; i1 < expectedValues.GetLength(0); i1++)
			{
				for (int i2 = 0; i2 < expectedValues.GetLength(1); i2++)
				{
					if (!comparer.AreEqual(expectedValues[i1, i2], computedValues[i1, i2]))
					{
						return false;
					}
				}
			}
			return true;
		}

		public static bool AreDisplacementsSame(double[] expectedValues,
			double[] computedValues)
		{
			var comparer = new ValueComparer(1E-14);
			for (int i1 = 0; i1 < expectedValues.GetLength(0); i1++)
			{
				if (!comparer.AreEqual(expectedValues[i1], computedValues[i1]))
				{
					return false;
				}
			}
			return true;
		}
	}
}
