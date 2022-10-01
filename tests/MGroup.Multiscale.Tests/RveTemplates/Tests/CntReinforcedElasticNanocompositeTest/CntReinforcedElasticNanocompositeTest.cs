namespace MGroup.Multiscale.Tests.RveTemplates.Tests.CntReinforcedElasticNanocompositeTest
{
	using System;
	using System.Collections.Generic;
	using System.Text;
	using MGroup.Constitutive.Structural.BoundaryConditions;
	using MGroup.Constitutive.Structural;
	using MGroup.NumericalAnalyzers;
	using MGroup.Solvers.Direct;
	using MGroup.Stochastic;

	using Xunit;
	using System.Linq;
	using MGroup.Constitutive.Structural.Continuum;
	using MGroup.LinearAlgebra.Matrices;
	using MGroup.MSolve.MultiscaleAnalysis;
	using MiMsolve.SolutionStrategies;
	using System.IO;

	public class CntReinforcedElasticNanocompositeTest
	{

		[Fact]
		public static void GenerateRVEwithCNTsSolutions2()
		{
			//LinearAlgebra.LibrarySettings.LinearAlgebraProviders = LinearAlgebra.LinearAlgebraProviderChoice.MKL;

			//var BasePath = Environment.GetFolderPath(Environment.SpecialFolder.Desktop);
			//var SpecPath = @"MsolveOutputs\matlabGeneratedCNTs\RVE Solutions";
			//var pathName = Path.Combine(BasePath, SpecPath);
			var SpecPath = "..\\..\\RveTemplates\\Input\\Output_Input Files";

			string InputFileName = "Input data.txt";
			string InputExtension = Path.GetExtension(InputFileName);
			string InputfileNameOnly = Path.Combine(SpecPath, Path.GetFileNameWithoutExtension(InputFileName));
			string inputFile = string.Format("{0}{1}", InputfileNameOnly, InputExtension);

			string OutputFileName = "Output data.txt";
			string OutputExtension = Path.GetExtension(OutputFileName);
			string OutputfileNameOnly = Path.Combine(SpecPath, Path.GetFileNameWithoutExtension(OutputFileName));
			string outputFile = string.Format("{0}{1}", OutputfileNameOnly, OutputExtension);

			bool append = false;

			int numberOfCnts = 50;
			int solutions = 1;
			int increments_per_solution = 5;
			double[][] Input = new double[solutions * increments_per_solution][];
			double[][] Output = new double[solutions * increments_per_solution][];

			var homogeneousRveBuilder1 =
				new CntReinforcedElasticNanocomposite(numberOfCnts); //{ K_el = 20, K_pl = 2, T_max = 0.2, };
			homogeneousRveBuilder1.readFromText = true;

			////TWO PHASE RVE BUILDER
			//var outterMaterial = new ExponentialDamageMaterial() { youngModulus = 20, poissonRatio = 0.2, Strain_0 = 0.001, A = 1, B = 500, Veta = 1 };
			////var outterMaterial = new BilinearDamageMaterial() { youngModulus = 20, poissonRatio = 0.2, Strain_0 = 0.002, Strain_f = 0.010, };
			////var outterMaterial = new SimpleExponentialModel();
			////var outterMaterial = new MazarsConcreteMaterial() { youngModulus = 20, poissonRatio = 0.2, At = 1.2, Bt = 15000, Ac = 1, Bc = 1500, Strain_0 = 0.0001, Veta = 1, };
			//var innerMaterial = new ElasticMaterial3D()
			//{ YoungModulus = 60, PoissonRatio = 0.22, };


			////WITH GMSH GEOMETRY DATA
			//var homogeneousRveBuilder1 =
			//    new GmshCompositeRveBuilder(outterMaterial, innerMaterial, 2, 2, 2, "..\\..\\..\\RveTemplates\\Input\\Continuum\\1ball.msh");

			IContinuumMaterial3D microstructure3 = new Microstructure3D<SkylineMatrix>(homogeneousRveBuilder1, false, 1, new SkylineSolverPrefernce());

			for (int num_solution = 0; num_solution < solutions; num_solution++)
			{
				var maxstrain = 0.1;
				var MacroStrain = new double[6] { maxstrain, -0.2 * maxstrain, -0.2 * maxstrain, 0.0, 0.0, 0.0 };
				//var MacroStrain = new double[6] { maxstrain, -0.2 * maxstrain, -0.2 * maxstrain, 0.05 * maxstrain, 0.05 * maxstrain, -0.05 * maxstrain };
				//var MacroStrain = new double[6]
				//{
				//0.00018213976876302329,
				//- 0.0040461599641024026,
				//0.031397465680760961,
				//- 0.004040226620050871,
				//0.0061003113176928171,
				//0.040854662588170977,
				//};
				//var MacroStrain = new double[6] { 0.0050, -0.0049, 0.0001, 0.0040, 0.0078, 0.0092 };
				//var MacroStrain = new double[6];  var trandom = new TRandom();
				//for (int ii = 0; ii < 6; ii++) MacroStrain[ii] = trandom.ContinuousUniform(-0.01, 0.01);
				//var sum_neg_strain = 0.0;
				//var sum_all_strain = 0.0;
				//for (int i = 0; i < 3; i++)
				//{
				//    if (MacroStrain[i] < 0)
				//    {
				//        sum_neg_strain += Math.Abs(MacroStrain[i]);
				//    }
				//    sum_all_strain += Math.Abs(MacroStrain[i]);
				//}
				//var ratio = sum_neg_strain / sum_all_strain + 0.05 * (1 - sum_neg_strain / sum_all_strain);
				//for (int ii = 0; ii < 6; ii++) MacroStrain[ii] = ratio * MacroStrain[ii];
				//MacroStrain[0] = -0.003403484; MacroStrain[1] = 0.000754793; MacroStrain[2] = -0.000982713; MacroStrain[3] = -0.002473461; MacroStrain[4] = 0.003131575; MacroStrain[5] = -0.003564857;
				//homogeneousRveBuilder1.K_el = trandom.ContinuousUniform(0.1, 20);
				//homogeneousRveBuilder1.K_pl = trandom.ContinuousUniform(0.01, 2);
				//homogeneousRveBuilder1.T_max = trandom.ContinuousUniform(0.001, 0.2);

				//homogeneousRveBuilder1.UpdateCohesiveMaterial();
				var IncrMacroStrainPrevious = new double[6];
				microstructure3 = new Microstructure3D<SkylineMatrix>(homogeneousRveBuilder1, false, 1, new SkylineSolverPrefernce());
				for (int i = 0; i < increments_per_solution; i++)
				{
					var IncrMacroStrain = new double[6];
					for (int ii = 0; ii < 6; ii++)
					{
						IncrMacroStrain[ii] = IncrMacroStrainPrevious[ii] + (MacroStrain[ii] / increments_per_solution);
						IncrMacroStrainPrevious[ii] = IncrMacroStrain[ii];
					}

					//{ IncrMacroStrain[ii] = MacroStrain[ii] * (i + 1) / increments_per_solution; }
					//var constitutive = microstructure3.ConstitutiveMatrix;
					double[] stresses = microstructure3.UpdateConstitutiveMatrixAndEvaluateResponse(new double[6] { IncrMacroStrain[0], IncrMacroStrain[1], IncrMacroStrain[2], IncrMacroStrain[3], IncrMacroStrain[4], IncrMacroStrain[5] });
					//Debug.WriteLine($"Strain {IncrMacroStrain[0]},{IncrMacroStrain[1]},{IncrMacroStrain[2]},{IncrMacroStrain[3]},{IncrMacroStrain[4]},{IncrMacroStrain[5]}");
					//microstructure3.UpdateMaterial(new double[6] { MacroStrain[0], MacroStrain[1], MacroStrain[2], MacroStrain[3], MacroStrain[4], MacroStrain[5] });

					double[] IncrMacroStress = new double[6] { stresses[0], stresses[1], stresses[2], stresses[3], stresses[4], stresses[5] };

					microstructure3.CreateState();
					Input[num_solution * increments_per_solution + i] = new double[6] { IncrMacroStrain[0], IncrMacroStrain[1], IncrMacroStrain[2], IncrMacroStrain[3], IncrMacroStrain[4], IncrMacroStrain[5] };
					Output[num_solution * increments_per_solution + i] = new double[6] { IncrMacroStress[0], IncrMacroStress[1], IncrMacroStress[2], IncrMacroStress[3], IncrMacroStress[4], IncrMacroStress[5] }; //homogeneousRveBuilder1.K_el, homogeneousRveBuilder1.K_pl, homogeneousRveBuilder1.T_max,

					using (var writer = new StreamWriter(inputFile, append)) // append mode to continue from previous increment
					{
						writer.WriteLine($"{Input[num_solution * increments_per_solution + i][0]}, {Input[num_solution * increments_per_solution + i][1]}, {Input[num_solution * increments_per_solution + i][2]}, " +
							$"{Input[num_solution * increments_per_solution + i][3]}, {Input[num_solution * increments_per_solution + i][4]}, {Input[num_solution * increments_per_solution + i][5]}");//, {Input[num_solution * increments_per_solution + i][6]}, " +
																																																	   //$"{Input[num_solution * increments_per_solution + i][7]}, {Input[num_solution * increments_per_solution + i][8]}");
					}

					using (var writer = new StreamWriter(outputFile, append)) // append mode to continue from previous increment
					{
						writer.WriteLine($"{Output[num_solution * increments_per_solution + i][0]}, {Output[num_solution * increments_per_solution + i][1]}, {Output[num_solution * increments_per_solution + i][2]}, " +
							$"{Output[num_solution * increments_per_solution + i][3]}, {Output[num_solution * increments_per_solution + i][4]}, {Output[num_solution * increments_per_solution + i][5]}");
					}
					append = true;
				}
			}
		}
	}
}
