using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using ISAAR.MSolve.MultiscaleAnalysis;


//using ISAAR.MSolve.Analyzers;
//using ISAAR.MSolve.Discretization;
//using ISAAR.MSolve.Discretization.Commons;
//using ISAAR.MSolve.Discretization.FreedomDegrees;
//using ISAAR.MSolve.LinearAlgebra.Matrices;
////using ISAAR.MSolve.FEM.Materials;

//using ISAAR.MSolve.LinearAlgebra.Output;
//using ISAAR.MSolve.Materials;
//using ISAAR.MSolve.Materials.Interfaces;
//using ISAAR.MSolve.MSAnalysis.RveTemplatesPaper;
//using ISAAR.MSolve.MultiscaleAnalysis;
//using ISAAR.MSolve.MultiscaleAnalysis.Interfaces;
////using ISAAR.MSolve.MSAnalysis.RveTemplatesPaper;
////using ISAAR.MSolve.MultiscaleAnalysis;
//using ISAAR.MSolve.Problems;
//using ISAAR.MSolve.Solvers;
//using ISAAR.MSolve.Solvers.Direct;
//using ISAAR.MSolve.Tests.FEMpartB;
//using JetBrains.dotMemoryUnit;
//using MathNet.Numerics.LinearAlgebra;

using MGroup.Constitutive.Structural.Continuum;
using MGroup.Constitutive.Structural.Shells;
using MGroup.LinearAlgebra.Commons;
using MGroup.LinearAlgebra.Matrices;
using MGroup.MSolve.MultiscaleAnalysis;
using MGroup.Multiscale;
using MGroup.Multiscale.RveTemplatesPaper;
using MGroup.Multiscale.SupportiveClasses;

using MiMsolve.SolutionStrategies;

//using Troschuetz.Random;

using Xunit;

//using MatlabWriter = MathNet.Numerics.Data.Matlab.MatlabWriter;

//[assembly: SuppressXUnitOutputException]

namespace ISAAR.MSolve.Tests
{
	public class RveTests
	{
		[Fact]
		public static void Check3DscaleTransitionsAndMicrostructure()
		{
			//MATERIAL LAW TO TEST WITH
			double E_disp = 3.5; /*Gpa*/ double ni_disp = 0.4; // stather Poisson
			var material1 = new ElasticMaterial3D(E_disp, ni_disp);
			double[] GLVec = new double[6] { 0.01, 0, 0, 0, 0, 0 };
			material1.UpdateConstitutiveMatrixAndEvaluateResponse(GLVec);
			double[] stressesCheck1 = new double[6] { material1.Stresses[0], material1.Stresses[1], material1.Stresses[2], material1.Stresses[3], material1.Stresses[4], material1.Stresses[5] };
			//material1.SaveState();
			GLVec = new double[6] { 0, 0, 0, 0, 0.02, 0 };
			material1.UpdateConstitutiveMatrixAndEvaluateResponse(GLVec);
			double[] stressesCheck2 = new double[6] { material1.Stresses[0], material1.Stresses[1], material1.Stresses[2], material1.Stresses[3], material1.Stresses[4], material1.Stresses[5] };


			//TWO PHASE RVE BUILDER
			var outterMaterial = new ElasticMaterial3D(E_disp, ni_disp);
			var innerMaterial = new ElasticMaterial3D(E_disp, ni_disp);


			//WITH GMSH GEOMETRY DATA
			var homogeneousRveBuilder1 =
				new GmshCompositeRveBuilder(outterMaterial, innerMaterial, 2, 2, 2, "..\\..\\RveTemplates\\Input\\Continuum\\1ball.msh");


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

			//COMPARISON
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
			double[] computedValues, double tol = 1E-14)
		{
			var comparer = new ValueComparer(tol);
			for (int i1 = 0; i1 < expectedValues.GetLength(0); i1++)
			{
				if (!comparer.AreEqual(expectedValues[i1], computedValues[i1]))
				{
					return false;
				}
			}
			return true;
		}

		//[Fact]
		//public void TestElasticAndMultiscaleMatricesTet()
		//{
		//	var material3 = new ShellElasticMaterial2Dtransformationb(youngModulus: 4.3210, poissonRation: 0.0)
		//	{
		//		TangentVectorV1 = new double[3] { 1, 0, 0 },
		//		TangentVectorV2 = new double[3] { 0, 1, 0 }
		//	};

		//	//var trandom = new TRandom();
		//	//var randomInnerE = trandom.Normal(3.4e9, 0.2e9);
		//	var outterMaterial = new ElasticMaterial3DTotalStrain(0, 4.3210);

		//	var innerMaterial = new ElasticMaterial3DTotalStrain(0, 34);


		//	var homogeneousRveBuilder1 =
		//		new CompositeMaterialModelBuilderTet2(outterMaterial, innerMaterial, 100, 100, 100);

		//	var material4 = new MicrostructureShell2D<SkylineMatrix>(homogeneousRveBuilder1, false, 1, new SkylineSolverPrefernce())
		//	{
		//		TangentVectorV1 = new double[3] { 1, 0, 0 },
		//		TangentVectorV2 = new double[3] { 0, 1, 0 }
		//	};
		//	material4.UpdateConstitutiveMatrixAndEvaluateResponse(new double[] { 0, 0, 0 });
		//}

		//[Fact]
		//public void CompositeRveWithElastic()
		//{
		//	var material3 = new ShellElasticMaterial2Dtransformationb(youngModulus: 4.3210, poissonRation: 0.0)
		//	{
		//		TangentVectorV1 = new double[3] { 1, 0, 0 },
		//		TangentVectorV2 = new double[3] { 0, 1, 0 }
		//	};

		//	//var trandom = new TRandom();
		//	//var randomInnerE = trandom.Normal(3.4e9, 0.2e9);
		//	var outterMaterial = new ElasticMaterial3DTotalStrain(0, 4.3210);

		//	var innerMaterial = new ElasticMaterial3DTotalStrain(0, 4.3210);


		//	var homogeneousRveBuilder1 =
		//		new GmshCompositeRveBuilder(outterMaterial, innerMaterial, 2, 2, 2, "..\\..\\..\\RveTemplates\\Input\\Continuum\\t16Solid_physical_entities_no_volume_tag_change_More_inclusions.msh");

		//	var material4 = new MicrostructureShell2D<SkylineMatrix>(homogeneousRveBuilder1, true, 1, new SkylineSolverPrefernce())
		//	{
		//		TangentVectorV1 = new double[3] { 1, 0, 0 },
		//		TangentVectorV2 = new double[3] { 0, 1, 0 }
		//	};
		//	//material4.UpdateMaterial(new double[] { 0, 0, 0 });

		//	var cons = material4.ConstitutiveMatrix;

		//	Assert.Equal(material3.ConstitutiveMatrix[0, 0], cons[0, 0], 7);
		//	Assert.Equal(material3.ConstitutiveMatrix[1, 1], cons[1, 1], 7);
		//	Assert.Equal(material3.ConstitutiveMatrix[2, 2], cons[2, 2], 7);
		//}


	}
}
