using System;
using System.Collections.Generic;
using System.Linq;
using System.Security.Cryptography;

using MGroup.Constitutive.Structural;
using MGroup.Constitutive.Structural.BoundaryConditions;
using MGroup.Constitutive.Structural.Cohesive;
using MGroup.Constitutive.Structural.Continuum;
using MGroup.FEM.Structural.Continuum;
using MGroup.LinearAlgebra.Commons;
using MGroup.MSolve.Discretization;
using MGroup.MSolve.Discretization.BoundaryConditions;
using MGroup.MSolve.Discretization.Dofs;
using MGroup.MSolve.Discretization.Entities;
using MGroup.MSolve.Numerics.Integration.Quadratures;
using MGroup.MSolve.Numerics.Interpolation;
using MGroup.MSolve.Solution;
using MGroup.Multiscale.SupportiveClasses;
using MGroup.NumericalAnalyzers;
using MGroup.NumericalAnalyzers.Discretization.NonLinear;
using MGroup.NumericalAnalyzers.Logging;
using MGroup.Solvers.Direct;

using Xunit;

namespace ISAAR.MSolve.SamplesConsole
{
    public static class CohesiveElementHexa8Interface
    {
        [Fact]
        public static void SolveCantilever()
        {
            Model model1 = new Model();
            model1.SubdomainsDictionary.Add(1, new Subdomain(1));
            Example_cohesive_hexa_orthi_constr_anw_benc1(model1,8);
            var log1 = RunAnalysis(model1);

            double[] displacements1 = new double[4] { 0.00019727473744350047, 0.00019727473744365822, 0.00019727473744356587, 0.0001972747374435206 };

			//double[] expected1 = new double[] { log1.GetTotalDisplacement(4, /*1*/model1.NodesDictionary[1], /*0*/StructuralDof.TranslationX), log1.GetTotalDisplacement(4, /*1*/model1.NodesDictionary[1], /*1*/StructuralDof.TranslationY), log1.GetTotalDisplacement(4, /*1*/model1.NodesDictionary[1], /*2*/StructuralDof.TranslationZ), log1.GetTotalDisplacement(4, /*1*/model1.NodesDictionary[1], /*3*/StructuralDof.RotationX) };
			double[] expected1 = new double[] { log1.GetTotalDisplacement(4, /*1*/model1.NodesDictionary[5], /*0*/StructuralDof.TranslationZ), log1.GetTotalDisplacement(4, /*1*/model1.NodesDictionary[6], /*1*/StructuralDof.TranslationZ), log1.GetTotalDisplacement(4, /*1*/model1.NodesDictionary[7], /*2*/StructuralDof.TranslationZ), log1.GetTotalDisplacement(4, /*1*/model1.NodesDictionary[8], /*3*/StructuralDof.TranslationZ) };
			Model model2 = new Model();
            model2.SubdomainsDictionary.Add(1, new Subdomain(1));
            Example_cohesive_hexa_orthi_constr_anw_benc1(model2, 13.75);
            var log2 = RunAnalysis(model2);


            double[] displacements2 = new double[4] { 0.00045445796734459886, 0.00045445796734467687, 0.00045445796734467741, 0.00045445796734453625 };
            //double[] expected2 = new double[] { log2.GetTotalDisplacement(4, /*1*/model1.NodesDictionary[1], /*0*/StructuralDof.TranslationX), log2.GetTotalDisplacement(4,/*1*/model1.NodesDictionary[1], /*1*/StructuralDof.TranslationY), log2.GetTotalDisplacement(4, /*1*/model1.NodesDictionary[1], /*2*/StructuralDof.TranslationZ), log2.GetTotalDisplacement(4, /*1*/model1.NodesDictionary[1], /*3*/StructuralDof.RotationX) };
			double[] expected2 = new double[] { log2.GetTotalDisplacement(4, /*1*/model2.NodesDictionary[5], /*0*/StructuralDof.TranslationZ), log2.GetTotalDisplacement(4, /*1*/model2.NodesDictionary[6], /*1*/StructuralDof.TranslationZ), log2.GetTotalDisplacement(4, /*1*/model2.NodesDictionary[7], /*2*/StructuralDof.TranslationZ), log2.GetTotalDisplacement(4, /*1*/model2.NodesDictionary[8], /*3*/StructuralDof.TranslationZ) };

			Assert.True(AreDisplacementsSame(expected1, displacements1));
            Assert.True(AreDisplacementsSame(expected2, displacements2));

        }

        private static bool AreDisplacementsSame(double[] expectedDisplacements, double[] computedDisplacements)//.
        {
            var comparer = new ValueComparer(1E-11);
            for (int i1 = 0; i1 < expectedDisplacements.Length; i1++)
            {
                if (!comparer.AreEqual(expectedDisplacements[i1], computedDisplacements[i1]))
                {
                    return false;
                }
            }//.
            return true;
        }

        public static TotalDisplacementsPerIterationLog RunAnalysis(Model model)
        {
            // Solver
            var solverFactory = new SkylineSolver.Factory();
			var algebraicModel = solverFactory.BuildAlgebraicModel(model);
			var solver = solverFactory.BuildSolver(algebraicModel);
			//ISolver solver = solverBuilder.BuildSolver(model);

            // Problem type
            var provider = new ProblemStructural(model, algebraicModel, solver);

            // Analyzers
            int increments = 2;
			var childAnalyzerBuilder = new LoadControlAnalyzer.Builder(model, algebraicModel, solver, provider, increments)
			{
				ResidualTolerance = 1E-8,
				MaxIterationsPerIncrement = 100,
				NumIterationsForMatrixRebuild = 1,
			};
			//var subdomainUpdaters = new[] { new NonLinearModelUpdater(algebraicModel) }; // This is the default
			LoadControlAnalyzer childAnalyzer = childAnalyzerBuilder.Build();
            var parentAnalyzer = new StaticAnalyzer(model, algebraicModel/*, solver*/, provider, childAnalyzer);

			// Output
			//var watchDofs = new Dictionary<int, int[]>();
			//watchDofs.Add(1, new int[] { 0, 1, 2, 3 });
			IList<(INode, IDofType)> watchDofs = new List<(INode, IDofType)>()
			{
				(model.NodesDictionary[5], StructuralDof.TranslationZ),
				(model.NodesDictionary[6], StructuralDof.TranslationZ),
				(model.NodesDictionary[7], StructuralDof.TranslationZ),
				(model.NodesDictionary[8], StructuralDof.TranslationZ),
			};

			var log1 = new TotalDisplacementsPerIterationLog(watchDofs, algebraicModel);
            childAnalyzer.TotalDisplacementsPerIterationLog = log1;

            parentAnalyzer.Initialize();
            parentAnalyzer.Solve();

            return log1;
        }

        //original:C:\Users\acivi\Documents\notes_elegxoi_2\develop_cohesive_tet\MSolve-0f36677a620c9801e578ad6e2ff6fc5ff80b2992_apo_links_internet
        //plhrofories: sto idio link apo panw
        public static void Example_cohesive_hexa_mixed(Model model)
        {
            CohesiveElementHexa8Interface.Example2Hexa8NL1Cohesive8node(model); 
            CohesiveElementHexa8Interface.Example2Hexa8NL1Cohesive8nodeConstraintsMixed(model, 1);
            //CohesiveElementHexa8Interface.Example2Hexa8NL1Cohesive8nodeLoadsMIxed(model, 1);
        }

        public static void Example_cohesive_hexa_orthi_elastic(Model model)
        {
            CohesiveElementHexa8Interface.Example2Hexa8NL1Cohesive8node(model); // me 1353000
            CohesiveElementHexa8Interface.Example2Hexa8NL1Cohesive8nodeConstraintsOrthiElastic(model, 3.47783);
            //CohesiveElementHexa8Interface.Example2Hexa8NL1Cohesive8nodeLoadsElasticOrthi(model, 3.47783);
        }

        public static void Example_cohesive_hexa_orthi_constr_anw_benc1(Model model, double load_value)
        {
            CohesiveElementHexa8Interface.Example2Hexa8NL1Cohesive8node(model); // me 135300
            CohesiveElementHexa8Interface.Example2Hexa8NL1Cohesive8nodeConstraintsBenc1(model, load_value);
            //ParadeigmataElegxwnBuilder.Example2Hexa8NL1Cohesive8nodeLoadsBenc1(model, 8);// gia elastiko klado 
            //ParadeigmataElegxwnBuilder.Example2Hexa8NL1Cohesive8nodeLoadsBenc1(model, 13.75); // gia metelastiko
            //CohesiveElementHexa8Interface.Example2Hexa8NL1Cohesive8nodeLoadsBenc1(model, load_value);// gia elastiko klado 
        }

        

        public static void Example2Hexa8NL1Cohesive8node(Model model)
        {
			ElasticMaterial3D material1 = new ElasticMaterial3D(135300, 0.3);
			//{
			//    YoungModulus = 135300, // 1353000 gia to allo paradeigma
			//    PoissonRatio = 0.3,
			//};
			var T_o_3 = 57;// N / mm2
			var D_o_3 = 0.000057; // mm
			var D_f_3 = 0.0098245610;  // mm

			var T_o_1 = 57;// N / mm2
			var D_o_1 = 0.000057; // mm
			var D_f_1 = 0.0098245610;  // mm

			var n_curve = 1.4;
			BenzeggaghKenaneCohesiveMaterial material2 = new BenzeggaghKenaneCohesiveMaterial(T_o_3, D_o_3, D_f_3, T_o_1, D_o_1, D_f_1, n_curve);

            double[,] nodeData = new double[,] {
            {0.500000,0.000000,1.000000},
            {0.500000,0.500000,1.000000},
            {0.000000,0.000000,1.000000},
            {0.000000,0.500000,1.000000},
            {0.500000,0.000000,0.500000},
            {0.500000,0.500000,0.500000},
            {0.000000,0.000000,0.500000},
            {0.000000,0.500000,0.500000},
            {0.500000,0.000000,0.000000},
            {0.500000,0.500000,0.000000},
            {0.000000,0.000000,0.000000},
            {0.000000,0.500000,0.000000},
            {0.500000,0.000000,0.500000},
            {0.500000,0.500000,0.500000},
            {0.000000,0.000000,0.500000},
            {0.000000,0.500000,0.500000} };

			//int[,] elementData = new int[,] {{1,1,2,4,3,5,6,8,7},{2,13,14,16,15,9,10,12,11},}
			int[,] elementData = new int[,] { { 2, 4, 3, 1, 6, 8, 7, 5 }, { 14, 16, 15, 13, 10, 12, 11, 9 }, };

			int[] cohesive8Nodes = new int[] { 5, 6, 8, 7, 13, 14, 16, 15, };

			// orismos shmeiwn
			for (int nNode = 0; nNode < nodeData.GetLength(0); nNode++)
            {
                model.NodesDictionary.Add(nNode+1, new Node(id:nNode+1, x: nodeData[nNode, 0], y: nodeData[nNode, 1], z: nodeData[nNode, 2]));

            }

            // orismos elements 
            IElementType e1;
            int subdomainID = 1;
			for (int nElement = 0; nElement < elementData.GetLength(0); nElement++)
            {
				List<INode> nodeSet = new List<INode>();
				for (int j = 0; j < 8; j++)
                {
                    int nodeID = elementData[nElement, j];
                    nodeSet.Add((Node)model.NodesDictionary[nodeID]);
				}
				//e1 = new Element()
				var factory = new MGroup.FEM.Structural.Continuum.ContinuumElement3DFactory(material1, null);
				e1 = factory.CreateNonLinearElement(CellType.Hexa8, nodeSet, material1.Clone(), null); /*new ContinuumElement3DNonLinear(nodeSet, material1, GaussLegendre3D.GetQuadratureWithOrder(3, 3, 3), InterpolationHexa8.UniqueInstance); // dixws to e. exoume sfalma enw sto beambuilding oxi//edw kaleitai me ena orisma to Hexa8*/

				//e1 = new ContinuumElement3DNonLinear(nodeSet, material1, GaussLegendre3D.GetQuadratureWithOrder(3, 3, 3), InterpolationHexa8.UniqueInstance);
				e1.ID = nElement + 1;
                    //ElementType = new ContinuumElement3DNonLinear(nodeSet, material1, GaussLegendre3D.GetQuadratureWithOrder(3, 3, 3), InterpolationHexa8Reversed.UniqueInstance) // dixws to e. exoume sfalma enw sto beambuilding oxi//edw kaleitai me ena orisma to Hexa8
                //for (int j = 0; j < 8; j++)
                //{
                //    e1.NodesDictionary.Add(elementData[nElement,j+1], model.NodesDictionary[elementData[nElement, j+1]]);
                //}
                model.ElementsDictionary.Add(e1.ID, e1);
                model.SubdomainsDictionary[subdomainID].Elements.Add(e1);
            }

			// kai to cohesive
			List<INode> nodeSetCohesive = new List<INode>();
			for (int j = 0; j < 8; j++)
			{
				nodeSetCohesive.Add(model.NodesDictionary[cohesive8Nodes[j]]);
			}
			e1 = new cohesiveElement(nodeSetCohesive, material2.Clone(), GaussLegendre2D.GetQuadratureWithOrder(3, 3), InterpolationQuad4.UniqueInstance); // dixws to e. exoume sfalma enw sto beambuilding oxi//edw kaleitai me ena orisma to Hexa8
			e1.ID = 3;
			//{
                //ID = 3,
                //ElementType = new cohesiveElement(material2, GaussLegendre2D.GetQuadratureWithOrder(3,3), InterpolationQuad4.UniqueInstance) // dixws to e. exoume sfalma enw sto beambuilding oxi//edw kaleitai me ena orisma to Hexa8
            //};
            //for (int j = 0; j < 8; j++)
            //{
            //    e1.NodesDictionary.Add(cohesive8Nodes[j], model.NodesDictionary[cohesive8Nodes[j]]);
            //}
            model.ElementsDictionary.Add(e1.ID, e1);
            model.SubdomainsDictionary[subdomainID].Elements.Add(e1);
        }

        public static void Example2Hexa8NL1Cohesive8nodeConstraintsOrthiElastic(Model model, double load_value)
        {
			var nodalDisplacementList = new List<NodalDisplacement>();
			for (int k = 13; k < 17; k++)
			{
				nodalDisplacementList.Add(new NodalDisplacement(model.NodesDictionary[k], StructuralDof.TranslationX, amount: 0));
				nodalDisplacementList.Add(new NodalDisplacement(model.NodesDictionary[k], StructuralDof.TranslationY, amount: 0));
				nodalDisplacementList.Add(new NodalDisplacement(model.NodesDictionary[k], StructuralDof.TranslationZ, amount: 0));
				//model.NodesDictionary[k].Constraints.Add(new Constraint { DOF = StructuralDof.TranslationX });
				//model.NodesDictionary[k].Constraints.Add(new Constraint { DOF = StructuralDof.TranslationY });
				//model.NodesDictionary[k].Constraints.Add(new Constraint { DOF = StructuralDof.TranslationZ });
			}
			var nodalLoadList = new List<NodalLoad>();
			for (int k = 1; k < 5; k++)
			{
				nodalLoadList.Add(new NodalLoad(model.NodesDictionary[k], StructuralDof.TranslationZ, amount: 1 * load_value));
				//load1 = new Load()
				//{
				//	Node = model.NodesDictionary[k],
				//	DOF = StructuralDof.TranslationZ,
				//	Amount = 1 * load_value

				//};
				//model.Loads.Add(load1);
			}
			model.BoundaryConditions.Add(new StructuralBoundaryConditionSet(nodalDisplacementList.ToArray(), nodalLoadList.ToArray()));

		}

        public static void Example2Hexa8NL1Cohesive8nodeConstraintsMixed(Model model, double load_value)
        {
			var nodalDisplacementList = new List<NodalDisplacement>();
			for (int k = 13; k < 17; k++)
			{
				nodalDisplacementList.Add(new NodalDisplacement(model.NodesDictionary[k], StructuralDof.TranslationX, amount: 0));
				nodalDisplacementList.Add(new NodalDisplacement(model.NodesDictionary[k], StructuralDof.TranslationY, amount: 0));
				nodalDisplacementList.Add(new NodalDisplacement(model.NodesDictionary[k], StructuralDof.TranslationZ, amount: 0));
				//model.NodesDictionary[k].Constraints.Add(new Constraint { DOF = StructuralDof.TranslationX });
				//model.NodesDictionary[k].Constraints.Add(new Constraint { DOF = StructuralDof.TranslationY });
				//model.NodesDictionary[k].Constraints.Add(new Constraint { DOF = StructuralDof.TranslationZ });
			}
			var nodalLoadList = new List<NodalLoad>();
			for (int k = 5; k < 9; k++)
			{
				nodalLoadList.Add(new NodalLoad(model.NodesDictionary[k], StructuralDof.TranslationX, amount: 1 * load_value));
				//load1 = new Load()
				//{
				//	Node = model.NodesDictionary[k],
				//	DOF = StructuralDof.TranslationX,
				//	Amount = 1 * load_value

				//};
				//model.Loads.Add(load1);

				nodalLoadList.Add(new NodalLoad(model.NodesDictionary[k], StructuralDof.TranslationZ, amount: 0.2 * load_value));
				//{
				//	Node = model.NodesDictionary[k],
				//	DOF = StructuralDof.TranslationZ,
				//	Amount = 0.2 * load_value

				//};
				//model.Loads.Add(load1);
			}
			model.BoundaryConditions.Add(new StructuralBoundaryConditionSet(nodalDisplacementList.ToArray(), nodalLoadList.ToArray()));
		}

        public static void Example2Hexa8NL1Cohesive8nodeConstraintsBenc1(Model model, double load_value)
        {
			var nodalDisplacementList = new List<NodalDisplacement>();
            for (int k = 1; k < 5; k++)
            {
				nodalDisplacementList.Add(new NodalDisplacement(model.NodesDictionary[k], StructuralDof.TranslationX, amount: 0));
				nodalDisplacementList.Add(new NodalDisplacement(model.NodesDictionary[k], StructuralDof.TranslationY, amount: 0));
				nodalDisplacementList.Add(new NodalDisplacement(model.NodesDictionary[k], StructuralDof.TranslationZ, amount: 0));
				//model.NodesDictionary[k].Constraints.Add(new Constraint { DOF = StructuralDof.TranslationX });
    //            model.NodesDictionary[k].Constraints.Add(new Constraint { DOF = StructuralDof.TranslationY });
    //            model.NodesDictionary[k].Constraints.Add(new Constraint { DOF = StructuralDof.TranslationZ });
            }
            for (int k = 5; k < 9; k++)
            {
				nodalDisplacementList.Add(new NodalDisplacement(model.NodesDictionary[k], StructuralDof.TranslationX, amount: 0));
				nodalDisplacementList.Add(new NodalDisplacement(model.NodesDictionary[k], StructuralDof.TranslationY, amount: 0));
				//model.NodesDictionary[k].Constraints.Add(new Constraint { DOF = StructuralDof.TranslationX });
    //            model.NodesDictionary[k].Constraints.Add(new Constraint { DOF = StructuralDof.TranslationY });
            }
            for (int k = 9; k < 17; k++)
            {
				nodalDisplacementList.Add(new NodalDisplacement(model.NodesDictionary[k], StructuralDof.TranslationX, amount: 0));
				nodalDisplacementList.Add(new NodalDisplacement(model.NodesDictionary[k], StructuralDof.TranslationY, amount: 0));
				nodalDisplacementList.Add(new NodalDisplacement(model.NodesDictionary[k], StructuralDof.TranslationZ, amount: 0));
				//model.NodesDictionary[k].Constraints.Add(new Constraint { DOF = StructuralDof.TranslationX });
    //            model.NodesDictionary[k].Constraints.Add(new Constraint { DOF = StructuralDof.TranslationY });
    //            model.NodesDictionary[k].Constraints.Add(new Constraint { DOF = StructuralDof.TranslationZ });
            }

			var nodalLoadList = new List<NodalLoad>();
			for (int k = 5; k < 9; k++)
			{
				nodalLoadList.Add(new NodalLoad(model.NodesDictionary[k], StructuralDof.TranslationZ, amount: 1 * load_value));
				//load1 = new Load()
				//{
				//	Node = model.NodesDictionary[k],
				//	DOF = StructuralDof.TranslationZ,
				//	Amount = 1 * load_value

				//};
				//model.Loads.Add(load1);
			}

			model.BoundaryConditions.Add(new StructuralBoundaryConditionSet(nodalDisplacementList.ToArray(), nodalLoadList.ToArray()));
		}

        //public static void Example2Hexa8NL1Cohesive8nodeLoadsMIxed(Model model, double load_value)
        //{
        //    Load load1;
        //    for (int k = 5; k < 9; k++)
        //    {
        //        load1 = new Load()
        //        {
        //            Node = model.NodesDictionary[k],
        //            DOF = StructuralDof.TranslationX,
        //            Amount = 1*load_value

        //        };
        //        model.Loads.Add(load1);
        //        load1 = new Load()
        //        {
        //            Node = model.NodesDictionary[k],
        //            DOF = StructuralDof.TranslationZ,
        //            Amount = 0.2*load_value

        //        };
        //        model.Loads.Add(load1);
        //    }
        //}

        //public static void Example2Hexa8NL1Cohesive8nodeLoadsElasticOrthi(Model model, double load_value)
        //{
        //    Load load1;
        //    for (int k = 1; k < 5; k++)
        //    {
        //        load1 = new Load()
        //        {
        //            Node = model.NodesDictionary[k],
        //            DOF = StructuralDof.TranslationZ,
        //            Amount = 1 * load_value

        //        };
        //        model.Loads.Add(load1);
        //    }
        //}

        //public static void Example2Hexa8NL1Cohesive8nodeLoadsBenc1(Model model, double load_value)
        //{
        //    Load load1;
        //    for (int k = 5; k < 9; k++)
        //    {
        //        load1 = new Load()
        //        {
        //            Node = model.NodesDictionary[k],
        //            DOF = StructuralDof.TranslationZ,
        //            Amount =1* load_value

        //        };
        //        model.Loads.Add(load1);
        //    }
        //}


        

    }
}
