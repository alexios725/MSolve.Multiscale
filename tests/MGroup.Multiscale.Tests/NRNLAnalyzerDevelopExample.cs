using MGroup.Constitutive.Structural;
using MGroup.Constitutive.Structural.BoundaryConditions;
using MGroup.Constitutive.Structural.Continuum;
using MGroup.Constitutive.Structural.Providers;
using MGroup.FEM.Structural.Continuum;
using MGroup.LinearAlgebra.Matrices;
using MGroup.MSolve.Discretization.Dofs;
using MGroup.MSolve.Discretization.Entities;
using MGroup.MSolve.MultiscaleAnalysis.SupportiveClasses;
using MGroup.MSolve.Numerics.Integration.Quadratures;
using MGroup.MSolve.Numerics.Interpolation;
using MGroup.MSolve.Solution;
using MGroup.MSolve.Solution.LinearSystem;
using MGroup.Multiscale.Tests.FEMpartB;
using MGroup.NumericalAnalyzers.Discretization.NonLinear;
using MGroup.Solvers.AlgebraicModel;
using MGroup.Solvers.Direct;
using MGroup.Solvers.DofOrdering;
using MGroup.Solvers.DofOrdering.Reordering;
using MiMsolve.multiScaleSupportiveClasses;
using System.Collections.Generic;
using Xunit;

namespace MGroup.Multiscale.Tests.ExampleModels
{
    public class NRNLAnalyzerDevelopExample
	{
        //public static ISolver exampleCreateSolver(Model model)
        //      {
        //	//var solverFactory = new SkylineSolver.Factory();
        //	//solverFactory.DofOrderer = new DofOrderer(new NodeMajorDofOrderingStrategy(), new NullReordering());
        //	//var algebraicModel = solverFactory.BuildAlgebraicModel(model);
        //	//ISolver solver = solverFactory.BuildSolver(algebraicModel);


        //}

        [Fact]
		public static void CheckMicrostructureBvpNRNLAnalyzer()
		{
			string results_file1 = "..\\..\\InputFiles\\MSmicroBvpAnalyzerTest\\U_sunol_micro_3.txt";
			string results_file2 = "..\\..\\InputFiles\\MSmicroBvpAnalyzerTest\\U_sunol_micro_6.txt";
			double[] displacements1sIncrement = PrintUtilities.ReadVector(results_file1);
			double[] displacements2ndncrement = PrintUtilities.ReadVector(results_file2);

			(var uInitialFreeDOFs_state1, var uInitialFreeDOFs_state2) = SolveDisplLoadsExample();


			bool Check1 = ComparisonMetods.AreDisplacementsSame(displacements1sIncrement, uInitialFreeDOFs_state1);
			bool Check2 = ComparisonMetods.AreDisplacementsSame(displacements2ndncrement, uInitialFreeDOFs_state2);
			
			Assert.True(Check1);
			Assert.True(Check2);
		}

		public static (double[], double[]) SolveDisplLoadsExample()
        {
			var model = HexaCantileverBuilderDispControl();
			model.ConnectDataStructures();
			ElementStructuralStiffnessProvider elementProvider = new ElementStructuralStiffnessProvider();
			//Dictionary<int, EquivalentContributionsAssebler> equivalentContributionsAssemblers = new Dictionary<int, EquivalentContributionsAssebler>();//SUNOLIKA STOIXEIA model.SubdomainsDictionary.Count oi oles tis model.subdomains ekei mallon deginontai access me ID.

			var solverFactory = new SkylineSolver.Factory();
			solverFactory.DofOrderer = new DofOrderer(new NodeMajorDofOrderingStrategy(), new NullReordering());
			var algebraicModel = solverFactory.BuildAlgebraicModel(model);
			ISolver solver = solverFactory.BuildSolver(algebraicModel);
			ProblemStructural provider = new ProblemStructural(model, algebraicModel, solver); //TODO ger mv1: to pou vrisketai h dhmiourgia tou problem structural prin to orderdofs dld
			algebraicModel.OrderDofs();

			//TODO Ger 45-50
			IGlobalVector forces = algebraicModel.CreateZeroVector();
			//algebraicModel.LinearSystem.RhsVector = forces;

			#region create boundary nodes and create displacements for 1st increment
			IGlobalVector uInitialFreeDOFDisplacementsPerSubdomain = algebraicModel.CreateZeroVector(); //new Dictionary<int, IVector>();
			//uInitialFreeDOFDisplacementsPerSubdomain.Add(model.SubdomainsDictionary[1].ID, Vector.CreateZero(44));//ordering1.NumGlobalFreeDofs prosoxh sto Id twn subdomain
			Dictionary<int, INode> boundaryNodes = new Dictionary<int, INode>();
			for (int k = 17; k < 21; k++)
			{
				boundaryNodes.Add(model.NodesDictionary[k].ID, (Node)model.NodesDictionary[k]);
			}
			Dictionary<int, Dictionary<IDofType, double>> initialConvergedBoundaryDisplacements = new Dictionary<int, Dictionary<IDofType, double>>();
			Dictionary<IDofType, double> initialConvergedBoundaryNodalDisplacements = new Dictionary<IDofType, double>();
			initialConvergedBoundaryNodalDisplacements.Add(StructuralDof.TranslationX, 0);
			for (int k = 17; k < 21; k++)
			{
				initialConvergedBoundaryDisplacements.Add(model.NodesDictionary[k].ID, initialConvergedBoundaryNodalDisplacements);
			}
			Dictionary<int, Dictionary<IDofType, double>> totalBoundaryDisplacements = new Dictionary<int, Dictionary<IDofType, double>>();
			double[] prescribedDisplacmentXValues = new double[4] { 7.81614E-01, 7.07355E-01, 7.81614E-01, 7.07355E-01 };
			for (int k = 17; k < 21; k++)
			{
				Dictionary<IDofType, double> totalBoundaryNodalDisplacements = new Dictionary<IDofType, double>();
				totalBoundaryNodalDisplacements.Add(StructuralDof.TranslationX, 0.5 * prescribedDisplacmentXValues[k - 17]);
				totalBoundaryDisplacements.Add(model.NodesDictionary[k].ID, totalBoundaryNodalDisplacements);
			}
			#endregion

			#region create nesessary structures and analyzers And Solve 1st increment            
			//ProblemStructural provider = new ProblemStructural(model, algebraicModel, solver);
			var subdomainUpdaters = new NonLinearModelUpdaterWithInitialConditions(algebraicModel);
			var increments = 1;

			var childAnalyzer = new DisplacementBvpNRNLAnalyzer(model, solver, subdomainUpdaters, provider, increments, uInitialFreeDOFDisplacementsPerSubdomain,
				boundaryNodes, initialConvergedBoundaryDisplacements, totalBoundaryDisplacements, algebraicModel);
			childAnalyzer.SetMaxIterations = 100;
			childAnalyzer.SetIterationsForMatrixRebuild = 1;

			MSParentAnalyzer parentAnalyzer = new MSParentAnalyzer(model, algebraicModel, solver, provider, childAnalyzer);
			//TODO MS
			//foreach (ILinearSystem linearSystem in solver.LinearSystems.Values)
			//{
			//    linearSystem.Subdomain.Forces = Vector.CreateZero(linearSystem.Size); // antistoixo tou subdomain.Forces = linearSystem.CreateZeroVector();
			//}
			parentAnalyzer.Initialize();
			parentAnalyzer.Solve();
			var array_uInitialFreeDOFs_state1 = RetrieveDisplacementsOfFreeDofs(algebraicModel, uInitialFreeDOFDisplacementsPerSubdomain);
			#endregion

			#region save state and update structures and vectors for second increment

			subdomainUpdaters.UpdateState(parentAnalyzer.CurrentState);
			
			// u (or uplusDu) initial 
			uInitialFreeDOFDisplacementsPerSubdomain = childAnalyzer.GetConvergedSolutionVectorsOfFreeDofs().Copy();// ousiastika to u pou twra taftizetai me to uPlusuu

			initialConvergedBoundaryDisplacements = totalBoundaryDisplacements;

			totalBoundaryDisplacements = new Dictionary<int, Dictionary<IDofType, double>>();
			for (int k = 17; k < 21; k++)
			{
				Dictionary<IDofType, double> totalBoundaryNodalDisplacements = new Dictionary<IDofType, double>();
				totalBoundaryNodalDisplacements.Add(StructuralDof.TranslationX, 1.0 * prescribedDisplacmentXValues[k - 17]);
				totalBoundaryDisplacements.Add(model.NodesDictionary[k].ID, totalBoundaryNodalDisplacements);
			}
			#endregion

			#region Creation of nessesary analyzers and solution 
			ElementStructuralStiffnessProvider elementProvider2 = new ElementStructuralStiffnessProvider();
			var solverBuilder2 = new SkylineSolver.Factory();
			solverBuilder2.DofOrderer = new DofOrderer(new NodeMajorDofOrderingStrategy(), new NullReordering());
			//ISolver solver2 = solverBuilder2.BuildSolver(algebraicModel);
			var solver2 = solver;
			//solver2.OrderDofs(false); TODO Ger1
			// anti gia thn parapanw endexomenws 
			//algebraicModel.OrderDofs();

			//TODO Ger1
			//foreach (ILinearSystem linearSystem in solver2.LinearSystems.Values) linearSystem.Reset();

			//kalutera apotelesmata otan to parakatw den kratietai, ara pragmati resets ta subd.forces
			//solver.ResetSubdomainForcesVector();

			//TODO Ger1
			//foreach (ILinearSystem linearSystem in linearSystems2.Values)
			//{
			//	linearSystem.RhsVector = linearSystem.Subdomain.Forces; //TODO MS 
			//}
			ProblemStructural provider2 = new ProblemStructural(model, algebraicModel, solver2);
			//var linearSystemsArray = new[] { linearSystems[1] };
			var subdomainUpdaters2 = new NonLinearModelUpdaterWithInitialConditions(algebraicModel);
			var increments2 = 1;

			var childAnalyzer2 = new DisplacementBvpNRNLAnalyzer(model, solver2, subdomainUpdaters2, provider2, increments2, uInitialFreeDOFDisplacementsPerSubdomain,
				boundaryNodes, initialConvergedBoundaryDisplacements, totalBoundaryDisplacements, algebraicModel);
			childAnalyzer2.SetMaxIterations = 100;
			childAnalyzer2.SetIterationsForMatrixRebuild = 1;

			MSParentAnalyzer parentAnalyzer2 = new MSParentAnalyzer(model, algebraicModel, solver2, provider2, childAnalyzer2);
			//parentAnalyzer2.BuildMatrices();
			//DdmCalculationsGeneral.UndoModelInterconnectionDataBuild(model);
			childAnalyzer2.Initialize(); //parentAnalyzer2.Initialize();
			parentAnalyzer2.Solve();
			var uInitialFreeDOFs_state2 = childAnalyzer2.GetConvergedSolutionVectorsOfFreeDofs().Copy();
			var array_uInitialFreeDOFs_state2 = RetrieveDisplacementsOfFreeDofs(algebraicModel, uInitialFreeDOFs_state2);
			#endregion

			return (array_uInitialFreeDOFs_state1, array_uInitialFreeDOFs_state2);
		}

		public static double[] RetrieveDisplacementsOfFreeDofs(GlobalAlgebraicModel<SkylineMatrix> globalAlgebraicModel,IGlobalVector uInitialFreeDOFDisplacementsPerSubdomain)
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


		public static Model HexaCantileverBuilderDispControl()
		{
			Model model = new Model();
			model.SubdomainsDictionary.Add(1, new Subdomain(1));

			var material1 = new ElasticMaterial3D(youngModulus: 1353000, poissonRatio: 0.3);

			double[,] nodeData = new double[,] { {-0.250000,-0.250000,-1.000000},
			{0.250000,-0.250000,-1.000000},
			{-0.250000,0.250000,-1.000000},
			{0.250000,0.250000,-1.000000},
			{-0.250000,-0.250000,-0.500000},
			{0.250000,-0.250000,-0.500000},
			{-0.250000,0.250000,-0.500000},
			{0.250000,0.250000,-0.500000},
			{-0.250000,-0.250000,0.000000},
			{0.250000,-0.250000,0.000000},
			{-0.250000,0.250000,0.000000},
			{0.250000,0.250000,0.000000},
			{-0.250000,-0.250000,0.500000},
			{0.250000,-0.250000,0.500000},
			{-0.250000,0.250000,0.500000},
			{0.250000,0.250000,0.500000},
			{-0.250000,-0.250000,1.000000},
			{0.250000,-0.250000,1.000000},
			{-0.250000,0.250000,1.000000},
			{0.250000,0.250000,1.000000}};

			int[,] elementData = new int[,] {{1,8,7,5,6,4,3,1,2},
			{2,12,11,9,10,8,7,5,6},
			{3,16,15,13,14,12,11,9,10},
			{4,20,19,17,18,16,15,13,14}, };

			// orismos shmeiwn
			for (int nNode = 0; nNode < nodeData.GetLength(0); nNode++)
			{
				model.NodesDictionary.Add(nNode + 1, new Node(id: nNode + 1, x: nodeData[nNode, 0], y: nodeData[nNode, 1], z: nodeData[nNode, 2]));

			}

			// orismos elements 
			int subdomainID = 1;
			for (int nElement = 0; nElement < elementData.GetLength(0); nElement++)
			{
				var nodeSet = new Node[8];
				for (var j = 0; j < 8; j++)
				{
					var nodeID = elementData[nElement, j + 1];
					nodeSet[j] = (Node)model.NodesDictionary[nodeID];
				}

				var e1 = new ContinuumElement3DNonLinear(nodeSet, material1, GaussLegendre3D.GetQuadratureWithOrder(orderXi: 3, orderEta: 3, orderZeta: 3),
					InterpolationHexa8.UniqueInstance)
				{
					ID = nElement + 1
				};
				model.ElementsDictionary.Add(e1.ID, e1);
				model.SubdomainsDictionary[subdomainID].Elements.Add(e1);
			}

			// constraint vashh opou z=-1
			var constraints = new List<INodalDisplacementBoundaryCondition>();
			for (int k = 1; k < 5; k++)
			{
				constraints.Add(new NodalDisplacement(model.NodesDictionary[k], StructuralDof.TranslationX, amount: 0d));
				constraints.Add(new NodalDisplacement(model.NodesDictionary[k], StructuralDof.TranslationY, amount: 0d));
				constraints.Add(new NodalDisplacement(model.NodesDictionary[k], StructuralDof.TranslationZ, amount: 0d));

			}

			// thetoume constraint tous prescribed
			//Load load1;
			for (int k = 17; k < 21; k++)
			{
				//load1 = new Load()
				//{
				//    Node = model.NodesDictionary[k],
				//    DOF = DOFType.X,
				//    Amount = 1 * load_value
				//};
				//model.Loads.Add(load1);
				constraints.Add(new NodalDisplacement(model.NodesDictionary[k], StructuralDof.TranslationX, amount: 0d));
			}

			model.BoundaryConditions.Add(new StructuralBoundaryConditionSet(constraints, new NodalLoad[] { }));

			return model;
		}

		public static Model CreateModel()
		{
			var nodeData = new double[,] {
				{-0.250000,-0.250000,-1.000000},
				{0.250000,-0.250000,-1.000000},
				{-0.250000,0.250000,-1.000000},
				{0.250000,0.250000,-1.000000},
				{-0.250000,-0.250000,-0.500000},
				{0.250000,-0.250000,-0.500000},
				{-0.250000,0.250000,-0.500000},
				{0.250000,0.250000,-0.500000},
				{-0.250000,-0.250000,0.000000},
				{0.250000,-0.250000,0.000000},
				{-0.250000,0.250000,0.000000},
				{0.250000,0.250000,0.000000},
				{-0.250000,-0.250000,0.500000},
				{0.250000,-0.250000,0.500000},
				{-0.250000,0.250000,0.500000},
				{0.250000,0.250000,0.500000},
				{-0.250000,-0.250000,1.000000},
				{0.250000,-0.250000,1.000000},
				{-0.250000,0.250000,1.000000},
				{0.250000,0.250000,1.000000}
			};

			var elementData = new int[,] {
				{1,8,7,5,6,4,3,1,2},
				{2,12,11,9,10,8,7,5,6},
				{3,16,15,13,14,12,11,9,10},
				{4,20,19,17,18,16,15,13,14}
			};

			var model = new Model();

			model.SubdomainsDictionary.Add(key: 0, new Subdomain(id: 0));

			for (var i = 0; i < nodeData.GetLength(0); i++)
			{
				var nodeId = i + 1;
				model.NodesDictionary.Add(nodeId, new Node(
					id: nodeId,
					x: nodeData[i, 0],
					y: nodeData[i, 1],
					z: nodeData[i, 2]
				));
			}

			for (var i = 0; i < elementData.GetLength(0); i++)
			{
				var nodeSet = new Node[8];
				for (var j = 0; j < 8; j++)
				{
					var nodeID = elementData[i, j + 1];
					nodeSet[j] = (Node)model.NodesDictionary[nodeID];
				}

				var element = new ContinuumElement3DNonLinear(
					nodeSet,
					new ElasticMaterial3D(youngModulus: 1353000, poissonRatio: 0.3),
					GaussLegendre3D.GetQuadratureWithOrder(orderXi: 3, orderEta: 3, orderZeta: 3),
					InterpolationHexa8.UniqueInstance
				)
				{
					ID = i + 1
				};

				model.ElementsDictionary.Add(element.ID, element);
				model.SubdomainsDictionary[0].Elements.Add(element);
			}

			var constraints = new List<INodalDisplacementBoundaryCondition>();
			for (var i = 1; i < 5; i++)
			{
				constraints.Add(new NodalDisplacement(model.NodesDictionary[i], StructuralDof.TranslationX, amount: 0d));
				constraints.Add(new NodalDisplacement(model.NodesDictionary[i], StructuralDof.TranslationY, amount: 0d));
				constraints.Add(new NodalDisplacement(model.NodesDictionary[i], StructuralDof.TranslationZ, amount: 0d));
			}

			var loads = new List<INodalLoadBoundaryCondition>();
			for (var i = 17; i < 21; i++)
			{
				loads.Add(new NodalLoad(model.NodesDictionary[i], StructuralDof.TranslationX, amount: 1 * 850d));
			}

			model.BoundaryConditions.Add(new StructuralBoundaryConditionSet(constraints, loads));

			return model;
		}

		public static IReadOnlyList<double[]> GetExpectedDisplacements()
		{
			var expectedDisplacements = new double[11][];

			expectedDisplacements[0] = new double[]
			{
				0.039075524153873623, -0.032541895181220408, -0.057387148941853101, -0.071994381984550326, -0.077053554770404833
			};
			expectedDisplacements[0] = new double[]
			{
				3.907552415387362300e-02, -3.254189518122040800e-02, -5.738714894185310100e-02, -7.199438198455032600e-02, -7.705355477040483300e-02
			};
			expectedDisplacements[1] = new double[]
			{
				4.061313406968563400e-02, -3.418876666892714500e-02, -6.682708262609965400e-02, -9.647418428408424700e-02, -1.214556593711370000e-01
			};
			expectedDisplacements[2] = new double[]
			{
				4.036171804663909300e-02, -3.396515033613205900e-02, -6.665084050819490600e-02, -9.713633946904017000e-02, -1.236631490430697600e-01
			};
			expectedDisplacements[3] = new double[]
			{
				4.032905162001462800e-02, -3.393260905426281900e-02, -6.657423779424630200e-02, -9.701032579889114200e-02, -1.234941821043235900e-01
			};
			expectedDisplacements[4] = new double[]
			{
				4.032900093364350700e-02, -3.393255831972321500e-02, -6.657411965268195100e-02, -9.701012513482368300e-02, -1.234939001150344400e-01
			};
			expectedDisplacements[5] = new double[]
			{
				8.095088461395548400e-02, -6.826589092291023000e-02, -1.393261307096994000e-01, -2.129883579558797000e-01, -2.840192458274605800e-01
			};
			expectedDisplacements[6] = new double[]
			{
				8.179065808895391600e-02, -6.914910025670165100e-02, -1.449912527358244700e-01, -2.283048858573358000e-01, -3.126785624370127000e-01
			};
			expectedDisplacements[7] = new double[]
			{
				8.008398180684392400e-02, -6.747544383562544000e-02, -1.408463169597064000e-01, -2.210877012127209200e-01, -3.022981704019522300e-01
			};
			expectedDisplacements[8] = new double[]
			{
				7.976397887674688300e-02, -6.715673915988762400e-02, -1.400151566610138300e-01, -2.195056794855129700e-01, -2.998365539162924900e-01
			};
			expectedDisplacements[9] = new double[]
			{
				7.975945236918889600e-02, -6.715223199537226400e-02, -1.400036710136937400e-01, -2.194845023343510200e-01, -2.998046100841828000e-01
			};
			expectedDisplacements[10] = new double[]
			{
				7.975944951878896600e-02, -6.715222916021290600e-02, -1.400036636464831200e-01, -2.194844883932760600e-01, -2.998045884933974200e-01
			};


			return expectedDisplacements;
		}
	}
}
