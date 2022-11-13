using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;

//using ISAAR.MSolve.FEM.Materials;
using ISAAR.MSolve.MultiscaleAnalysis;
//using ISAAR.MSolve.MultiscaleAnalysis;
//using ISAAR.MSolve.MultiscaleAnalysis.Interfaces;
//using MathNet.Numerics.Data.Matlab;
//using MathNet.Numerics.LinearAlgebra;

using MGroup.Constitutive.Structural.Cohesive;
using MGroup.Constitutive.Structural.Continuum;
using MGroup.LinearAlgebra.Commons;
using MGroup.LinearAlgebra.Matrices;
using MGroup.MSolve.MultiscaleAnalysis;
using MGroup.MSolve.Solution.AlgebraicModel;
using MGroup.MSolve.Solution.LinearSystem;
using MGroup.Solvers.AlgebraicModel;
using MGroup.Solvers.Direct;

using MiMsolve.SolutionStrategies;

//using Troschuetz.Random;
using Xunit;

namespace ISAAR.MSolve.Tests.PaperPreliminary
{
    public class CohesiveRveLevelGmshV2
    {
        private const double Tolerance = 1e-9;

        [Fact]// FINISHED V2 DAMAGE test
        public void TestCohesiveV2ElementsDamageUnloadingV2()// this was run for the results of  diagramm hyperelastic plane stress ...One_RVER_example.m
        {
            //var outterMaterial = new ElasticMaterial3D() { PoissonRatio = 0.3, YoungModulus = 135300 };//.

            //var innerMaterial = new ElasticMaterial3D() { PoissonRatio = 0.3, YoungModulus = 135300 };//.



            double T_o_3 = 57 * 10;// N / mm2
            double D_o_3 = 2.5 * 8 * 10 * 5.7e-5; // mm
            double D_f_3 = 10 * 10 * 0.009824561;  // mm
            double T_o_1 = 10 * 57;// N / mm2
            double D_o_1 = 2.5 * 8 * 10 * 5.7e-5; // mm
            double D_f_1 = 10 * 10 * 0.009824561;  // mm
            double n_curve = 1.4;

			ICohesiveZoneMaterial cohesiveMate = new BenzeggaghKenaneCohesiveMaterial(T_o_3, D_o_3, D_f_3, T_o_1, D_o_1, D_f_1, n_curve);
            //{
            //    T_o_3 = 57 * 10,// N / mm2
            //    D_o_3 = 2.5 * 8 * 10 * 5.7e-5, // mm
            //    D_f_3 = 10 * 10 * 0.009824561,  // mm

            //    T_o_1 = 10 * 57,// N / mm2
            //    D_o_1 = 2.5 * 8 * 10 * 5.7e-5, // mm
            //    D_f_1 = 10 * 10 * 0.009824561,  // mm

            //    n_curve = 1.4
            //};

            //ICohesiveZoneMaterial3D cohesiveMate = new BondSlipCohMat(1.25 * T_o_1, 1.25 * D_o_1, 0.1, T_o_3, D_o_3, new double[2], new double[2], 1e-10);




            var homogenousRveHyper = new GmshCompositeRveBuilderNL_DefGradCohesive(
				new ElasticMaterial3DDefGrad(135300,0.3) /*{ PoissonRatio = 0.3, YoungModulus = 135300 }*/,
                new ElasticMaterial3DDefGrad(135300, 0.3) /*{ PoissonRatio = 0.3, YoungModulus = 135300 }*/,
                2, 2, 2, cohesiveMate, "..\\..\\RveTemplates\\Input\\Continuum\\cylinders.msh");


            var materialHyper = new MicrostructureDefGrad2D<SkylineMatrix>(homogenousRveHyper, false, 1, new SkylineSolverPrefernce()
                /*model1 => (new SuiteSparseSolver.Builder()).BuildSolver(model1)*/);


            int loadsteps = 3; // 3*4;
            int cycles = 2;

            double[][] stressesHist = new double[cycles * loadsteps][];
            double[][,] consHist = new double[cycles * loadsteps][,];

            double[][] uplhsduHist = new double[cycles * loadsteps][];

            double[] strain1 = new double[] { 1.04, 1.04, 0, 0 };
            materialHyper.UpdateConstitutiveMatrixAndEvaluateResponse(strain1);
            materialHyper.CreateState();
            var stresses1 = new double[4]; materialHyper.Stresses.CopyTo(stresses1, 0);
            var cons1 = new double[4, 4];
            for (int i1 = 0; i1 < 4; i1++)
            {
                for (int i2 = 0; i2 < 4; i2++)
                {
                    cons1[i1, i2] = materialHyper.ConstitutiveMatrix[i1, i2];
				}
            }
			var solution1 = RetrieveDisplacementsOfFreeDofs(materialHyper.globalAlgebraicModel, materialHyper.uInitialFreeDOFDisplacementsPerSubdomain.Copy());
			//var solution1 = materialHyper.uInitialFreeDOFDisplacementsPerSubdomain[0].CopyToArray();

			double[] strain2 = new double[] { 1.06, 1.06, 0, 0 };
            materialHyper.UpdateConstitutiveMatrixAndEvaluateResponse(strain2);
            materialHyper.CreateState();

            double[] strain3 = new double[] { 1.04, 1.04, 0, 0 };
            materialHyper.UpdateConstitutiveMatrixAndEvaluateResponse(strain3);
            materialHyper.CreateState();
            var stresses3 = new double[4]; materialHyper.Stresses.CopyTo(stresses3, 0);
            var cons3 = new double[4, 4];
            for (int i1 = 0; i1 < 4; i1++)
            {
                for (int i2 = 0; i2 < 4; i2++)
                {
                    cons3[i1, i2] = materialHyper.ConstitutiveMatrix[i1, i2];
                }
            }
			var solution3 = RetrieveDisplacementsOfFreeDofs(materialHyper.globalAlgebraicModel, materialHyper.uInitialFreeDOFDisplacementsPerSubdomain.Copy());
			//var solution3 = materialHyper.uInitialFreeDOFDisplacementsPerSubdomain[0].CopyToArray();


            var stresses1Expected = new double[] { 5441.571611903363, 5434.433052021342, 8.173947264248, 8.162923992561 };
            var cons1expected = new double[4, 4] {{109097.544878695320,22417.423516881401,117.079810128881,109.239564599948},
{22417.423516881416,108739.403431449930,116.318015674788,126.062017496355},
{109.239564599927,126.062017496324,34827.373758082431,40058.306201180094},
{117.079810128768,116.318015674798,40053.424277864993,34827.373758082424}};

            var stresses3expected = new double[] { 5229.2966263925691, 5222.3104837633, 8.2719511477282062, 8.269972229435151 };

            var cons3expected = new double[4, 4] {{112366.142007139390,25946.563379919029,100.444990454461,92.557316439333},
{25946.563379919047,112172.932179793960,112.892439463564,119.775325389351},
{92.557316439327,119.775325389306,34786.500624240478,39814.816488005046},
{100.444990454530,112.892439463553,39808.426883538690,34786.500624240485} };

            Assert.True(AreDisplacementsSame(stresses1, stresses1Expected));
            Assert.True(AreDisplacementsSame(cons1, cons1expected));
            Assert.True(AreDisplacementsSame(stresses3, stresses3expected, 1e-11));
            Assert.True(AreDisplacementsSame(cons3, cons3expected, 1e-11));

        }

        [Fact] // FINISHED V2 DUCTILE test
        public void TestCohesiveV1ElementsDuctileV2()// this was run for the results of  diagramm hyperelastic plane stress ...One_RVER_example.m
        {
            //var outterMaterial = new ElasticMaterial3D() { PoissonRatio = 0.3, YoungModulus = 135300 };//.

            //var innerMaterial = new ElasticMaterial3D() { PoissonRatio = 0.3, YoungModulus = 135300 };//.



            double T_o_3 = 57 * 10;// N / mm2
            double D_o_3 = 2.5 * 8 * 10 * 5.7e-5; // mm
            double D_f_3 = 10 * 10 * 0.009824561;  // mm
            double T_o_1 = 10 * 57;// N / mm2
            double D_o_1 = 2.5 * 8 * 10 * 5.7e-5; // mm
            double D_f_1 = 10 * 10 * 0.009824561;  // mm
            double n_curve = 1.4;

            //ICohesiveZoneMaterial3D cohesiveMate = new BenzeggaghKenaneCohesiveMaterial()
            //{
            //    T_o_3 = 57 * 10,// N / mm2
            //    D_o_3 = 2.5 * 8 * 10 * 5.7e-5, // mm
            //    D_f_3 = 10 * 10 * 0.009824561,  // mm

            //    T_o_1 = 10 * 57,// N / mm2
            //    D_o_1 = 2.5 * 8 * 10 * 5.7e-5, // mm
            //    D_f_1 = 10 * 10 * 0.009824561,  // mm

            //    n_curve = 1.4
            //};

            ICohesiveZoneMaterial cohesiveMate = new BondSlipMaterial(1.25 * T_o_1, 1.25 * D_o_1, 0.1, T_o_3, D_o_3, new double[2], new double[2], 1e-10);




            var homogenousRveHyper = new GmshCompositeRveBuilderNL_DefGradCohesive(
                new ElasticMaterial3DDefGrad(135300, 0.3) /*{ PoissonRatio = 0.3, YoungModulus = 135300 }*/,
                new ElasticMaterial3DDefGrad(135300, 0.3) /*{ PoissonRatio = 0.3, YoungModulus = 135300 }*/,
                2, 2, 2, cohesiveMate, "..\\..\\RveTemplates\\Input\\Continuum\\cylinders.msh");


            var materialHyper = new MicrostructureDefGrad2D<SkylineMatrix>(homogenousRveHyper, false, 1, new SkylineSolverPrefernce()
				/*model1 => (new SuiteSparseSolver.Builder()).BuildSolver(model1), false, 1*/);


            int loadsteps = 3; // 3*4;
            int cycles = 2;

            var consInit = new double[4, 4];
            for (int i1 = 0; i1 < 4; i1++)
            {
                for (int i2 = 0; i2 < 4; i2++)
                {
                    consInit[i1, i2] = materialHyper.ConstitutiveMatrix[i1, i2];
                }
            }

            double[][] stressesHist = new double[cycles * loadsteps][];
            double[][,] consHist = new double[cycles * loadsteps][,];

            double[][] uplhsduHist = new double[cycles * loadsteps][];

            double[] strain1 = new double[] { 1.04, 1.04, 0, 0 };
            materialHyper.UpdateConstitutiveMatrixAndEvaluateResponse(strain1);
            materialHyper.CreateState();
            var stresses1 = new double[4]; materialHyper.Stresses.CopyTo(stresses1, 0);
            var cons1 = new double[4, 4];
            for (int i1 = 0; i1 < 4; i1++)
            {
                for (int i2 = 0; i2 < 4; i2++)
                {
                    cons1[i1, i2] = materialHyper.ConstitutiveMatrix[i1, i2];
                }
            }
			var solution1 = RetrieveDisplacementsOfFreeDofs(materialHyper.globalAlgebraicModel, materialHyper.uInitialFreeDOFDisplacementsPerSubdomain.Copy());
			//var solution1 = materialHyper.uInitialFreeDOFDisplacementsPerSubdomain[0].CopyToArray();

            double[] strain2 = new double[] { 1.06, 1.06, 0, 0 };
            materialHyper.UpdateConstitutiveMatrixAndEvaluateResponse(strain2);
            materialHyper.CreateState();

            double[] strain3 = new double[] { 1.04, 1.04, 0, 0 };
            materialHyper.UpdateConstitutiveMatrixAndEvaluateResponse(strain3);
            materialHyper.CreateState();
            var stresses3 = new double[4]; materialHyper.Stresses.CopyTo(stresses3, 0);
            var cons3 = new double[4, 4];
            for (int i1 = 0; i1 < 4; i1++)
            {
                for (int i2 = 0; i2 < 4; i2++)
                {
                    cons3[i1, i2] = materialHyper.ConstitutiveMatrix[i1, i2];
                }
            }
			var solution3 = RetrieveDisplacementsOfFreeDofs(materialHyper.globalAlgebraicModel, materialHyper.uInitialFreeDOFDisplacementsPerSubdomain.Copy());
			//var solution3 = materialHyper.uInitialFreeDOFDisplacementsPerSubdomain[0].CopyToArray();


            var stresses1Expected = new double[] { 5624.0246345210217, 5619.1856134736681, 5.4652603319164657, 5.4661592275691575 };
            var cons1expected = new double[4, 4] {{120151.688379432210,27457.768343336047,109.372284523065,104.599375509946},
{27457.768343336087,120035.626480634780,109.851627287372,115.008987102358},
{104.599375509948,115.008987102321,38311.264409130665,43719.007583742292},
{109.372284523083,109.851627287378,43714.436082074535,38311.264409130643}};

            var stresses3expected = new double[] { 5550.8486456556648, 5545.9116420674454, 7.4529490769122235, 7.4637762280841109 };

            var cons3expected = new double[4, 4] {{120487.302301225010,27803.052505472027,65.577984797856,60.762037876704},
{27803.052505471969,120352.150429660110,72.543396686939,77.206554533504},
{60.762037876717,77.206554533531,38499.259285159002,43839.382022526530},
{65.577984797853,72.543396686918,43834.352743039737,38499.259285158965}};

            Assert.True(AreDisplacementsSame(stresses1, stresses1Expected));
            Assert.True(AreDisplacementsSame(cons1, cons1expected));
            Assert.True(AreDisplacementsSame(stresses3, stresses3expected));
            Assert.True(AreDisplacementsSame(cons3, cons3expected, 1e-11));

        }

        [Fact]
        public void TestCohesiveRveDefGradDamageUnloadingV1()// this was run for the results of  diagramm hyperelastic plane stress ...One_RVER_example.m
        {
            //var outterMaterial = new ElasticMaterial3D() { PoissonRatio = 0.3, YoungModulus = 135300 };//.

            //var innerMaterial = new ElasticMaterial3D() { PoissonRatio = 0.3, YoungModulus = 135300 };//.



            double T_o_3 = 57 * 10;// N / mm2
            double D_o_3 = 2.5 * 8 * 10 * 5.7e-5; // mm
            double D_f_3 = 10 * 10 * 0.009824561;  // mm
            double T_o_1 = 10 * 57;// N / mm2
            double D_o_1 = 2.5 * 8 * 10 * 5.7e-5; // mm
            double D_f_1 = 10 * 10 * 0.009824561;  // mm
            double n_curve = 1.4;

			ICohesiveZoneMaterial cohesiveMate = new BenzeggaghKenaneCohesiveMaterial(T_o_3, D_o_3, D_f_3, T_o_1, D_o_1, D_f_1, n_curve);
            //{
            //    T_o_3 = 57 * 10,// N / mm2
            //    D_o_3 = 2.5 * 8 * 10 * 5.7e-5, // mm
            //    D_f_3 = 10 * 10 * 0.009824561,  // mm

            //    T_o_1 = 10 * 57,// N / mm2
            //    D_o_1 = 2.5 * 8 * 10 * 5.7e-5, // mm
            //    D_f_1 = 10 * 10 * 0.009824561,  // mm

            //    n_curve = 1.4
            //};

            //ICohesiveZoneMaterial3D cohesiveMate = new BondSlipCohMat(1.25 * T_o_1, 1.25 * D_o_1, 0.1, T_o_3, D_o_3, new double[2], new double[2], 1e-10);




            var homogenousRveHyper = new GmshCompositeRveBuilderNL_DefGradCohesive(
                new ElasticMaterial3DDefGrad(135300, 0.3) /*{ PoissonRatio = 0.3, YoungModulus = 135300 }*/,
                new ElasticMaterial3DDefGrad(135300, 0.3) /*{ PoissonRatio = 0.3, YoungModulus = 135300 }*/,
                2, 2, 2, cohesiveMate, "..\\..\\RveTemplates\\Input\\Continuum\\cylinders.msh");


            var materialHyper = new MicrostructureDefGrad2D<SkylineMatrix>(homogenousRveHyper, false, 1, new SkylineSolverPrefernce()
				/*model1 => (new SuiteSparseSolver.Builder()).BuildSolver(model1), false, 1*/);


            int loadsteps = 3; // 3*4;
            int cycles = 2;

            double[][] stressesHist = new double[cycles * loadsteps][];
            double[][,] consHist = new double[cycles * loadsteps][,];

            double[][] uplhsduHist = new double[cycles * loadsteps][];

            double[] strain1 = new double[] { 1.04, 1.04, 0, 0 };
            materialHyper.UpdateConstitutiveMatrixAndEvaluateResponse(strain1);
            materialHyper.CreateState();
            var stresses1 = new double[4]; materialHyper.Stresses.CopyTo(stresses1, 0);
            var cons1 = new double[4, 4];
            for (int i1 = 0; i1 < 4; i1++)
            {
                for (int i2 = 0; i2 < 4; i2++)
                {
                    cons1[i1, i2] = materialHyper.ConstitutiveMatrix[i1, i2];
                }
            }
			var solution1 = RetrieveDisplacementsOfFreeDofs(materialHyper.globalAlgebraicModel, materialHyper.uInitialFreeDOFDisplacementsPerSubdomain.Copy());
			//var solution1 = materialHyper.uInitialFreeDOFDisplacementsPerSubdomain[0].CopyToArray();

            double[] strain2 = new double[] { 1.06, 1.06, 0, 0 };
            materialHyper.UpdateConstitutiveMatrixAndEvaluateResponse(strain2);
            materialHyper.CreateState();

            double[] strain3 = new double[] { 1.04, 1.04, 0, 0 };
            materialHyper.UpdateConstitutiveMatrixAndEvaluateResponse(strain3);
            materialHyper.CreateState();
            var stresses3 = new double[4]; materialHyper.Stresses.CopyTo(stresses3, 0);
            var cons3 = new double[4, 4];
            for (int i1 = 0; i1 < 4; i1++)
            {
                for (int i2 = 0; i2 < 4; i2++)
                {
                    cons3[i1, i2] = materialHyper.ConstitutiveMatrix[i1, i2];
                }
            }
			var solution3 = RetrieveDisplacementsOfFreeDofs(materialHyper.globalAlgebraicModel, materialHyper.uInitialFreeDOFDisplacementsPerSubdomain.Copy());
			//var solution3 = materialHyper.uInitialFreeDOFDisplacementsPerSubdomain[0].CopyToArray();


            var stresses1Expected = new double[] { 5441.571611903363, 5434.433052021342, 8.173947264248, 8.162923992561 };
            var cons1expected = new double[4, 4] {{109097.544878695320,22417.423516881401,117.079810128881,109.239564599948},
{22417.423516881416,108739.403431449930,116.318015674788,126.062017496355},
{109.239564599927,126.062017496324,34827.373758082431,40058.306201180094},
{117.079810128768,116.318015674798,40053.424277864993,34827.373758082424}};

            var stresses3expected = new double[] { 5229.2966263925691, 5222.3104837633, 8.2719511477282062, 8.269972229435151 };

            var cons3expected = new double[4, 4] {{112366.142007139390,25946.563379919029,100.444990454461,92.557316439333},
{25946.563379919047,112172.932179793960,112.892439463564,119.775325389351},
{92.557316439327,119.775325389306,34786.500624240478,39814.816488005046},
{100.444990454530,112.892439463553,39808.426883538690,34786.500624240485} };

            Assert.True(AreDisplacementsSame(stresses1, stresses1Expected));
            Assert.True(AreDisplacementsSame(cons1, cons1expected));
            Assert.True(AreDisplacementsSame(stresses3, stresses3expected,1e-12));
            
			Assert.True(AreDisplacementsSame(cons3, cons3expected,1e-11));

        }

        [Fact] 
        public void TestCohesiveRveDefGradDuctileV1()// this was run for the results of  diagramm hyperelastic plane stress ...One_RVER_example.m
        {
            //var outterMaterial = new ElasticMaterial3D() { PoissonRatio = 0.3, YoungModulus = 135300 };//.

            //var innerMaterial = new ElasticMaterial3D() { PoissonRatio = 0.3, YoungModulus = 135300 };//.



            double T_o_3 = 57 * 10;// N / mm2
            double D_o_3 = 2.5 * 8 * 10 * 5.7e-5; // mm
            double D_f_3 = 10 * 10 * 0.009824561;  // mm
            double T_o_1 = 10 * 57;// N / mm2
            double D_o_1 = 2.5 * 8 * 10 * 5.7e-5; // mm
            double D_f_1 = 10 * 10 * 0.009824561;  // mm
            double n_curve = 1.4;

            //ICohesiveZoneMaterial3D cohesiveMate = new BenzeggaghKenaneCohesiveMaterial()
            //{
            //    T_o_3 = 57 * 10,// N / mm2
            //    D_o_3 = 2.5 * 8 * 10 * 5.7e-5, // mm
            //    D_f_3 = 10 * 10 * 0.009824561,  // mm

            //    T_o_1 = 10 * 57,// N / mm2
            //    D_o_1 = 2.5 * 8 * 10 * 5.7e-5, // mm
            //    D_f_1 = 10 * 10 * 0.009824561,  // mm

            //    n_curve = 1.4
            //};

            ICohesiveZoneMaterial cohesiveMate = new BondSlipMaterial(1.25 * T_o_1, 1.25 * D_o_1, 0.1, T_o_3, D_o_3, new double[2], new double[2], 1e-10);




            var homogenousRveHyper = new GmshCompositeRveBuilderNL_DefGradCohesive(
                new ElasticMaterial3DDefGrad(135300, 0.3) /*{ PoissonRatio = 0.3, YoungModulus = 135300 }*/,
                new ElasticMaterial3DDefGrad(135300, 0.3) /*{ PoissonRatio = 0.3, YoungModulus = 135300 }*/,
                2, 2, 2, cohesiveMate, "..\\..\\RveTemplates\\Input\\Continuum\\cylinders.msh");


            var materialHyper = new MicrostructureDefGrad2D<SkylineMatrix>(homogenousRveHyper, false, 1, new SkylineSolverPrefernce()
				/*model1 => (new SuiteSparseSolver.Builder()).BuildSolver(model1)*/);


            int loadsteps = 3; // 3*4;
            int cycles = 2;

            var consInit = new double[4, 4];
            for (int i1 = 0; i1 < 4; i1++)
            {
                for (int i2 = 0; i2 < 4; i2++)
                {
                    consInit[i1, i2] = materialHyper.ConstitutiveMatrix[i1, i2];
                }
            }

            double[][] stressesHist = new double[cycles * loadsteps][];
            double[][,] consHist = new double[cycles * loadsteps][,];

            double[][] uplhsduHist = new double[cycles * loadsteps][];

            double[] strain1 = new double[] { 1.04, 1.04, 0, 0 };
            materialHyper.UpdateConstitutiveMatrixAndEvaluateResponse(strain1);
            materialHyper.CreateState();
            var stresses1 = new double[4]; materialHyper.Stresses.CopyTo(stresses1, 0);
            var cons1 = new double[4, 4];
            for (int i1 = 0; i1 < 4; i1++)
            {
                for (int i2 = 0; i2 < 4; i2++)
                {
                    cons1[i1, i2] = materialHyper.ConstitutiveMatrix[i1, i2];
                }
            }
			var solution1 = RetrieveDisplacementsOfFreeDofs(materialHyper.globalAlgebraicModel, materialHyper.uInitialFreeDOFDisplacementsPerSubdomain.Copy());
			//var solution1 = materialHyper.uInitialFreeDOFDisplacementsPerSubdomain[0].CopyToArray();

            double[] strain2 = new double[] { 1.06, 1.06, 0, 0 };
            materialHyper.UpdateConstitutiveMatrixAndEvaluateResponse(strain2);
            materialHyper.CreateState();

            double[] strain3 = new double[] { 1.04, 1.04, 0, 0 };
            materialHyper.UpdateConstitutiveMatrixAndEvaluateResponse(strain3);
            materialHyper.CreateState();
            var stresses3 = new double[4]; materialHyper.Stresses.CopyTo(stresses3, 0);
            var cons3 = new double[4, 4];
            for (int i1 = 0; i1 < 4; i1++)
            {
                for (int i2 = 0; i2 < 4; i2++)
                {
                    cons3[i1, i2] = materialHyper.ConstitutiveMatrix[i1, i2];
                }
            }
			var solution3 = RetrieveDisplacementsOfFreeDofs(materialHyper.globalAlgebraicModel, materialHyper.uInitialFreeDOFDisplacementsPerSubdomain.Copy());
			//var solution3 = materialHyper.uInitialFreeDOFDisplacementsPerSubdomain[0].CopyToArray();


            var stresses1Expected = new double[] { 5624.0246345210217, 5619.1856134736681, 5.4652603319164657, 5.4661592275691575 };
            var cons1expected = new double[4, 4] {{120151.688379432210,27457.768343336047,109.372284523065,104.599375509946},
{27457.768343336087,120035.626480634780,109.851627287372,115.008987102358},
{104.599375509948,115.008987102321,38311.264409130665,43719.007583742292},
{109.372284523083,109.851627287378,43714.436082074535,38311.264409130643}};

            var stresses3expected = new double[] { 5550.8486456556648, 5545.9116420674454, 7.4529490769122235, 7.4637762280841109 };

            var cons3expected = new double[4, 4] {{120487.302301225010,27803.052505472027,65.577984797856,60.762037876704},
{27803.052505471969,120352.150429660110,72.543396686939,77.206554533504},
{60.762037876717,77.206554533531,38499.259285159002,43839.382022526530},
{65.577984797853,72.543396686918,43834.352743039737,38499.259285158965}};

            Assert.True(AreDisplacementsSame(stresses1, stresses1Expected));
            Assert.True(AreDisplacementsSame(cons1, cons1expected));
            Assert.True(AreDisplacementsSame(stresses3, stresses3expected));
            Assert.True(AreDisplacementsSame(cons3, cons3expected,1e-11));

        }


        [Fact]
        public void TestCohesiveRveSmallStrainDamageUnloading1()// this was run for the results of  diagramm hyperelastic plane stress ...One_RVER_example.m
        {
			var outterMaterial = new ElasticMaterial3D(135300, 0.3);/* { PoissonRatio = 0.3, YoungModulus = 135300 }*///.

			var innerMaterial = new ElasticMaterial3D(135300, 0.3);/* { PoissonRatio = 0.3, YoungModulus = 135300 };//.*/



            double T_o_3 = 57 * 10;// N / mm2
            double D_o_3 = 2.5 * 8 * 10 * 5.7e-5; // mm
            double D_f_3 = 10 * 10 * 0.009824561;  // mm
            double T_o_1 = 10 * 57;// N / mm2
            double D_o_1 = 2.5 * 8 * 10 * 5.7e-5; // mm
            double D_f_1 = 10 * 10 * 0.009824561;  // mm
            double n_curve = 1.4;

			ICohesiveZoneMaterial cohesiveMate = new BenzeggaghKenaneCohesiveMaterial(T_o_3, D_o_3, D_f_3, T_o_1, D_o_1, D_f_1, n_curve);
            //{
            //    T_o_3 = 57 * 10,// N / mm2
            //    D_o_3 = 2.5 * 8 * 10 * 5.7e-5, // mm
            //    D_f_3 = 10 * 10 * 0.009824561,  // mm

            //    T_o_1 = 10 * 57,// N / mm2
            //    D_o_1 = 2.5 * 8 * 10 * 5.7e-5, // mm
            //    D_f_1 = 10 * 10 * 0.009824561,  // mm

            //    n_curve = 1.4
            //};

            //ICohesiveZoneMaterial3D cohesiveMate = new BondSlipCohMat(1.25*T_o_1, 1.25*D_o_1, 0.1, T_o_3, D_o_3, new double[2], new double[2], 1e-10);




            var homogenousRveHyper =
                new GmshCompositeRveBuilderNLCohesive(outterMaterial, innerMaterial, 2, 2, 2, cohesiveMate, "..\\..\\RveTemplates\\Input\\Continuum\\cylinders.msh");
            var materialHyper = new MicrostructureDefGrad2D<SkylineMatrix>(homogenousRveHyper, false, 1, new SkylineSolverPrefernce()
				/*model1 => (new SuiteSparseSolver.Builder()).BuildSolver(model1)*/);


            int loadsteps = 3; // 3*4;
            int cycles = 2;

            double[][] stressesHist = new double[cycles * loadsteps][];
            double[][,] consHist = new double[cycles * loadsteps][,];

            double[][] uplhsduHist = new double[cycles * loadsteps][];

            double[] strain1 = new double[] { 1.04, 1.04, 0, 0 };
            materialHyper.UpdateConstitutiveMatrixAndEvaluateResponse(strain1);
            materialHyper.CreateState();
            var stresses1 = new double[4]; materialHyper.Stresses.CopyTo(stresses1, 0);
            var cons1 = new double[4, 4];
            for (int i1 = 0; i1 < 4; i1++)
            {
                for (int i2 = 0; i2 < 4; i2++)
                {
                    cons1[i1, i2] = materialHyper.ConstitutiveMatrix[i1, i2];
                }
            }
			var solution1 = RetrieveDisplacementsOfFreeDofs(materialHyper.globalAlgebraicModel, materialHyper.uInitialFreeDOFDisplacementsPerSubdomain.Copy());
			//var solution1 = materialHyper.uInitialFreeDOFDisplacementsPerSubdomain[0].CopyToArray();

            double[] strain2 = new double[] { 1.06, 1.06, 0, 0 };
            materialHyper.UpdateConstitutiveMatrixAndEvaluateResponse(strain2);
            materialHyper.CreateState();

            double[] strain3 = new double[] { 1.04, 1.04, 0, 0 };
            materialHyper.UpdateConstitutiveMatrixAndEvaluateResponse(strain3);
            materialHyper.CreateState();
            var stresses3 = new double[4]; materialHyper.Stresses.CopyTo(stresses3, 0);
            var cons3 = new double[4, 4];
            for (int i1 = 0; i1 < 4; i1++)
            {
                for (int i2 = 0; i2 < 4; i2++)
                {
                    cons3[i1, i2] = materialHyper.ConstitutiveMatrix[i1, i2];
                }
            }
			var solution3 = RetrieveDisplacementsOfFreeDofs(materialHyper.globalAlgebraicModel, materialHyper.uInitialFreeDOFDisplacementsPerSubdomain.Copy());
			//var solution3 = materialHyper.uInitialFreeDOFDisplacementsPerSubdomain[0].CopyToArray();


            var stresses1Expected = new double[] { 5441.571611903363, 5434.433052021342, 8.173947264248, 8.162923992561 };
            var cons1expected = new double[4, 4] {{109097.544878695320,22417.423516881401,117.079810128881,109.239564599948},
{22417.423516881416,108739.403431449930,116.318015674788,126.062017496355},
{109.239564599927,126.062017496324,34827.373758082431,40058.306201180094},
{117.079810128768,116.318015674798,40053.424277864993,34827.373758082424}};

            var stresses3expected = new double[] { 5229.2966263925691, 5222.3104837633, 8.2719511477282062, 8.269972229435151 };

            var cons3expected = new double[4, 4] {{112366.142007139390,25946.563379919029,100.444990454461,92.557316439333},
{25946.563379919047,112172.932179793960,112.892439463564,119.775325389351},
{92.557316439327,119.775325389306,34786.500624240478,39814.816488005046},
{100.444990454530,112.892439463553,39808.426883538690,34786.500624240485} };

            Assert.True(AreDisplacementsSame(stresses1, stresses1Expected));
            Assert.True(AreDisplacementsSame(cons1, cons1expected));
            Assert.True(AreDisplacementsSame(stresses3, stresses3expected));
            Assert.True(AreDisplacementsSame(cons3, cons3expected));

        }

        [Fact]
        public void TestCohesiveRveSmallStrainDuctileUnloading1()// this was run for the results of  diagramm hyperelastic plane stress ...One_RVER_example.m
        {


			var outterMaterial = new ElasticMaterial3D(135300, 0.3);/* { PoissonRatio = 0.3, YoungModulus = 135300 };//.*/

			var innerMaterial = new ElasticMaterial3D(135300, 0.3);/* { PoissonRatio = 0.3, YoungModulus = 135300 };//.*/



            double T_o_3 = 57 * 10;// N / mm2
            double D_o_3 = 2.5 * 8 * 10 * 5.7e-5; // mm
            double D_f_3 = 10 * 10 * 0.009824561;  // mm
            double T_o_1 = 10 * 57;// N / mm2
            double D_o_1 = 2.5 * 8 * 10 * 5.7e-5; // mm
            double D_f_1 = 10 * 10 * 0.009824561;  // mm
            double n_curve = 1.4;

            //ICohesiveZoneMaterial3D cohesiveMate = new BenzeggaghKenaneCohesiveMaterial()
            //{
            //    T_o_3 = 57 * 10,// N / mm2
            //    D_o_3 = 2.5 * 8 * 10 * 5.7e-5, // mm
            //    D_f_3 = 10 * 10 * 0.009824561,  // mm

            //    T_o_1 = 10 * 57,// N / mm2
            //    D_o_1 = 2.5 * 8 * 10 * 5.7e-5, // mm
            //    D_f_1 = 10 * 10 * 0.009824561,  // mm

            //    n_curve = 1.4
            //};

            ICohesiveZoneMaterial cohesiveMate = new BondSlipMaterial(1.25*T_o_1, 1.25*D_o_1, 0.1, T_o_3, D_o_3, new double[2], new double[2], 1e-10);




            var homogenousRveHyper =
                new GmshCompositeRveBuilderNLCohesive(outterMaterial, innerMaterial, 2, 2, 2, cohesiveMate, "..\\..\\RveTemplates\\Input\\Continuum\\cylinders.msh");
            var materialHyper = new MicrostructureDefGrad2D<SkylineMatrix>(homogenousRveHyper, false, 1, new SkylineSolverPrefernce()
				/*model1 => (new SuiteSparseSolver.Builder()).BuildSolver(model1)*/);


            int loadsteps = 3; // 3*4;
            int cycles = 2;

            double[][] stressesHist = new double[cycles * loadsteps][];
            double[][,] consHist = new double[cycles * loadsteps][,];

            double[][] uplhsduHist = new double[cycles * loadsteps][];

            double[] strain1 = new double[] { 1.04, 1.04, 0, 0 };
            materialHyper.UpdateConstitutiveMatrixAndEvaluateResponse(strain1);
            materialHyper.CreateState();
            var stresses1 = new double[4]; materialHyper.Stresses.CopyTo(stresses1, 0);
            var cons1 = new double[4, 4];
            for (int i1 = 0; i1 < 4; i1++)
            {
                for (int i2 = 0; i2 < 4; i2++)
                {
                    cons1[i1, i2] = materialHyper.ConstitutiveMatrix[i1, i2];
                }
            }
			var solution1 = RetrieveDisplacementsOfFreeDofs(materialHyper.globalAlgebraicModel, materialHyper.uInitialFreeDOFDisplacementsPerSubdomain.Copy());
			//var solution1 = materialHyper.uInitialFreeDOFDisplacementsPerSubdomain[0].CopyToArray();

            double[] strain2 = new double[] { 1.06, 1.06, 0, 0 };
            materialHyper.UpdateConstitutiveMatrixAndEvaluateResponse(strain2);
            materialHyper.CreateState();

            double[] strain3 = new double[] { 1.04, 1.04, 0, 0 };
            materialHyper.UpdateConstitutiveMatrixAndEvaluateResponse(strain3);
            materialHyper.CreateState();
            var stresses3 = new double[4]; materialHyper.Stresses.CopyTo(stresses3, 0);
            var cons3 = new double[4, 4];
            for (int i1 = 0; i1 < 4; i1++)
            {
                for (int i2 = 0; i2 < 4; i2++)
                {
                    cons3[i1, i2] = materialHyper.ConstitutiveMatrix[i1, i2];
                }
            }
			var solution3 = RetrieveDisplacementsOfFreeDofs(materialHyper.globalAlgebraicModel, materialHyper.uInitialFreeDOFDisplacementsPerSubdomain.Copy());
			//var solution3 = materialHyper.uInitialFreeDOFDisplacementsPerSubdomain[0].CopyToArray();


            var stresses1Expected = new double[] { 5624.0246345210217, 5619.1856134736681, 5.4652603319164657, 5.4661592275691575 };
            var cons1expected = new double[4, 4] {{120151.688379432210,27457.768343336047,109.372284523065,104.599375509946},
{27457.768343336087,120035.626480634780,109.851627287372,115.008987102358},
{104.599375509948,115.008987102321,38311.264409130665,43719.007583742292},
{109.372284523083,109.851627287378,43714.436082074535,38311.264409130643}};

            var stresses3expected = new double[] { 5550.8486456556648, 5545.9116420674454, 7.4529490769122235, 7.4637762280841109 };

            var cons3expected = new double[4, 4] {{120487.302301225010,27803.052505472027,65.577984797856,60.762037876704},
{27803.052505471969,120352.150429660110,72.543396686939,77.206554533504},
{60.762037876717,77.206554533531,38499.259285159002,43839.382022526530},
{65.577984797853,72.543396686918,43834.352743039737,38499.259285158965}};

            Assert.True(AreDisplacementsSame(stresses1, stresses1Expected));
            Assert.True(AreDisplacementsSame(cons1, cons1expected));
            Assert.True(AreDisplacementsSame(stresses3, stresses3expected));
            Assert.True(AreDisplacementsSame(cons3, cons3expected));

        }

        private static bool AreDisplacementsSame(double[,] cons1, double[,] cons1Expected, double tol = 1e-11)
        {
            var comparer = new ValueComparer(tol);
            for (int iter = 0; iter < cons1.GetLength(0); ++iter)
            {
                for (int iter2 = 0; iter2 < cons1.GetLength(0); ++iter2)
                {
                    if (!comparer.AreEqual(cons1[iter,iter2], cons1Expected[iter,iter2]))
                    {
                        return false;
                    }
                }
            }
            return true;
        }

        private static bool AreDisplacementsSame(double[] cons1, double[] cons1Expected, double tol = 1e-11)
        {
            var comparer = new ValueComparer(tol);
            for (int iter = 0; iter < cons1.GetLength(0); ++iter)
            {
                
                    if (!comparer.AreEqual(cons1[iter], cons1Expected[iter]))
                    {
                        return false;
                    }
                
            }
            return true;
        }

		public static double[] RetrieveDisplacementsOfFreeDofs(GlobalAlgebraicModel<SkylineMatrix> globalAlgebraicModel, IGlobalVector uInitialFreeDOFDisplacementsPerSubdomain)
		{
			var uInitialFreeDOFs_state1_Data = globalAlgebraicModel.ExtractAllResults(uInitialFreeDOFDisplacementsPerSubdomain);
			double[] uInitialFreeDOFs_state1_array = new double[globalAlgebraicModel.SubdomainFreeDofOrdering.NumFreeDofs];
			int counter = 0;
			foreach ((int node, int dof, int freeDofIdx) in globalAlgebraicModel.SubdomainFreeDofOrdering.FreeDofs)
			{
				uInitialFreeDOFs_state1_array[counter] = uInitialFreeDOFs_state1_Data.Data[node, dof];
				counter++;
			}


			return uInitialFreeDOFs_state1_array;
		}





	}
}
