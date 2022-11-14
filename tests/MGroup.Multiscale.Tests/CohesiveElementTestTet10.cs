using System;
using System.Collections.Generic;
using System.Linq;
using System.Security.Cryptography;

using ISAAR.MSolve.FEM.Elements;
using ISAAR.MSolve.FEM.Embedding;

using MGroup.Constitutive.Structural;
using MGroup.Constitutive.Structural.BoundaryConditions;
using MGroup.Constitutive.Structural.Cohesive;
using MGroup.LinearAlgebra.Vectors;
using MGroup.MSolve.Discretization;
using MGroup.MSolve.Discretization.BoundaryConditions;
using MGroup.MSolve.Discretization.Entities;
using MGroup.MSolve.Discretization.Providers;
using MGroup.MSolve.Numerics.Integration.Quadratures;
using MGroup.MSolve.Numerics.Interpolation;
using MGroup.MSolve.Solution.AlgebraicModel;
using MGroup.MSolve.Solution.LinearSystem;
using MGroup.Multiscale.SupportiveClasses;
using MGroup.NumericalAnalyzers;
using MGroup.NumericalAnalyzers.Discretization.NonLinear;
using MGroup.NumericalAnalyzers.Logging;
using MGroup.Solvers.AlgebraicModel;
using MGroup.Solvers.Direct;
using MGroup.Solvers.LinearSystem;

using Xunit;

namespace ISAAR.MSolve.SamplesConsole
{
    public static class CohesiveElementTestTet10
    {
		[Fact]
		public static void SolveInterface()
		{
			int subdomainID = 1;
			const int increments = 2;
			const double nodalDisplacement = -5.0;

			// Create Model
			Model model = new Model();

			// Create Subdomain
			model.SubdomainsDictionary.Add(subdomainID, new Subdomain(subdomainID));

			var T_o_3 = 57;// N / mm2
			var D_o_3 = 0.000057; // mm
			var D_f_3 = 0.0098245610;  // mm

			var T_o_1 = 57;// N / mm2
			var D_o_1 = 0.000057; // mm
			var D_f_1 = 0.0098245610;  // mm

			var n_curve = 1.4;
			BenzeggaghKenaneCohesiveMaterial material2 = new BenzeggaghKenaneCohesiveMaterial(T_o_3, D_o_3, D_f_3, T_o_1, D_o_1, D_f_1, n_curve);

			int[] nodeIds = new int[] { 5, 6, 7, 8, 17, 18, 19, 20, 38, 39, 40, 41, 42 };

			double[,] nodeData = new double[,]
				{{-0.2500000000000000,0.2500000000000000,1.0000000000000000},
				{0.2500000000000000,0.2500000000000000,1.0000000000000000},
				{0.2500000000000000,-0.2500000000000000,1.0000000000000000},
				{-0.2500000000000000,-0.2500000000000000,1.0000000000000000},

				{0.0000000000000000,0.2500000000000000,1.0000000000000000},
				{0.2500000000000000,0.0000000000000000,1.0000000000000000},
				{0.0000000000000000,-0.2500000000000000,1.0000000000000000},
				{-0.2500000000000000,0.0000000000000000,1.0000000000000000},

				{0.0000000000000000,0.0000000000000000,1.0000000000000000},
				{-0.1250000000000000,0.1250000000000000,1.0000000000000000},
				{0.1250000000000000,0.1250000000000000,1.0000000000000000},
				{0.1250000000000000,-0.1250000000000000,1.0000000000000000},
				{-0.1250000000000000,-0.1250000000000000,1.0000000000000000},};

			int[,] elementData = new int[,]
			{
					{6,5,38,17,39,40 },
					{5,8,38,20,42,39 },
					{7,6,38,18,40,41 },
					{8,7,38,19,41,42 }
			};

			int[] upperSideSuplicateNodeIds = nodeIds.Select(x => x + 100).ToArray();

			// orismos shmeiwn
			for (int nNode = 0; nNode < nodeIds.GetLength(0); nNode++)
			{
				model.NodesDictionary.Add(nodeIds[nNode], new Node(id: nodeIds[nNode], x: nodeData[nNode, 0], y: nodeData[nNode, 1], z: nodeData[nNode, 2]));

			}
			for (int nNode = 0; nNode < upperSideSuplicateNodeIds.GetLength(0); nNode++)
			{
				model.NodesDictionary.Add(upperSideSuplicateNodeIds[nNode], new Node(id: upperSideSuplicateNodeIds[nNode], x: nodeData[nNode, 0], y: nodeData[nNode, 1], z: nodeData[nNode, 2]));

			}

			// orismos elements 
			IElementType e1;

			for (int nElement = 0; nElement < elementData.GetLength(0); nElement++)
			{
				List<INode> nodeSet = new List<INode>();
				for (int j = 0; j < 6; j++)
				{
					nodeSet.Add(model.NodesDictionary[elementData[nElement, j] + 100]);
				}
				for (int j = 0; j < 6; j++)
				{
					nodeSet.Add(model.NodesDictionary[elementData[nElement, j]]);
				}
				e1 = new cohesiveElement(nodeSet, material2, /*GaussLegendre2D.GetQuadratureWithOrder(3, 3)*/ TriangleQuadratureSymmetricGaussian.Order2Points3, InterpolationTri6.UniqueInstance); // dixws to e. exoume sfalma enw sto beambuilding oxi//edw kaleitai me ena orisma to Hexa8
				e1.ID = nElement + 1;
				//{
				//	ID = nElement + 1,
				//	ElementType = new cohesiveElement(nodeSet,material2, /*GaussLegendre2D.GetQuadratureWithOrder(3, 3)*/ TriangleQuadratureSymmetricGaussian.Order2Points3, InterpolationTri6.UniqueInstance) // dixws to e. exoume sfalma enw sto beambuilding oxi//edw kaleitai me ena orisma to Hexa8
				//};
				//for (int j = 0; j < 6; j++)
				//{
				//	e1.NodesDictionary.Add(elementData[nElement, j] + 100, model.NodesDictionary[elementData[nElement, j] + 100]);
				//}
				//for (int j = 0; j < 6; j++)
				//{
				//	e1.NodesDictionary.Add(elementData[nElement, j], model.NodesDictionary[elementData[nElement, j]]);
				//}
				model.ElementsDictionary.Add(e1.ID, e1);
				model.SubdomainsDictionary[subdomainID].Elements.Add(e1);
			}

			// Boundary Condtitions
			List<NodalDisplacement> nodalDisplacementList = new List<NodalDisplacement>();
			for (int i1 = 0; i1 < nodeIds.GetLength(0); i1++)
			{
				nodalDisplacementList.Add(new NodalDisplacement(model.NodesDictionary[nodeIds[i1]], StructuralDof.TranslationX, amount: 0));
				nodalDisplacementList.Add(new NodalDisplacement(model.NodesDictionary[nodeIds[i1]], StructuralDof.TranslationY, amount: 0));
				nodalDisplacementList.Add(new NodalDisplacement(model.NodesDictionary[nodeIds[i1]], StructuralDof.TranslationZ, amount: 0));
				//model.NodesDictionary[nodeIds[i1]].Constraints.Add(new Constraint { DOF = StructuralDof.TranslationX });
				//model.NodesDictionary[nodeIds[i1]].Constraints.Add(new Constraint { DOF = StructuralDof.TranslationY });
				//model.NodesDictionary[nodeIds[i1]].Constraints.Add(new Constraint { DOF = StructuralDof.TranslationZ });
			}
			//model.BoundaryConditions.Add(new StructuralBoundaryConditionSet(nodalDisplacementList.ToArray(), null));
			model.BoundaryConditions.Add(new StructuralBoundaryConditionSet(nodalDisplacementList.ToArray(), null));
			//for (int i1 = 0; i1 < upperSideSuplicateNodeIds.GetLength(0)-1; i1++)
			//{
			//	model.NodesDictionary[upperSideSuplicateNodeIds[i1]].Constraints.Add(new Constraint { DOF = StructuralDof.TranslationX });
			//	model.NodesDictionary[upperSideSuplicateNodeIds[i1]].Constraints.Add(new Constraint { DOF = StructuralDof.TranslationY });
			//	model.NodesDictionary[upperSideSuplicateNodeIds[i1]].Constraints.Add(new Constraint { DOF = StructuralDof.TranslationZ, Amount = 0.0001 });
			//}

			//model.NodesDictionary[upperSideSuplicateNodeIds[upperSideSuplicateNodeIds.GetLength(0) - 1]].Constraints.Add(new Constraint { DOF = StructuralDof.TranslationX });
			//model.NodesDictionary[upperSideSuplicateNodeIds[upperSideSuplicateNodeIds.GetLength(0) - 1]].Constraints.Add(new Constraint { DOF = StructuralDof.TranslationY });
			//model.NodesDictionary[upperSideSuplicateNodeIds[upperSideSuplicateNodeIds.GetLength(0) - 1]].Constraints.Add(new Constraint { DOF = StructuralDof.TranslationZ, Amount = 0.0001 });

			var solution = Vector.CreateFromArray(new double[upperSideSuplicateNodeIds.Count() * 3]);
			var dSolution = Vector.CreateFromArray(new double[upperSideSuplicateNodeIds.Count() * 3]);

			//model.SubdomainsDictionary.ElementAt(0).Value.GetRhsFromSolution(solution, dSolution);

			for (int i1 = 0; i1 < upperSideSuplicateNodeIds.GetLength(0); i1++)
			{
				solution[3 * i1 + 2] = 0.00019727473744350047;
			}


			//model.ConnectDataStructures();



			// Choose linear equation system solver
			var solverFactory = new SkylineSolver.Factory();
			var algebraicModel = solverFactory.BuildAlgebraicModel(model);
			var solver = solverFactory.BuildSolver(algebraicModel);
			//SkylineSolver solver = solverBuilder.BuildSolver(model);

			// Choose the provider of the problem -> here a structural problem
			var provider = new ProblemStructural(model, algebraicModel, solver);

			// Choose child analyzer -> Child: DisplacementControlAnalyzer
			var subdomainUpdaters = new[] { new NonLinearModelUpdater(algebraicModel) };
			var childAnalyzerBuilder = new DisplacementControlAnalyzer.Builder(model, algebraicModel, solver, provider, increments)
			{
				MaxIterationsPerIncrement = 100,
				NumIterationsForMatrixRebuild = 1,
				ResidualTolerance = 1E-03
			};
			var childAnalyzer = childAnalyzerBuilder.Build();

			// Choose parent analyzer -> Parent: Static
			var parentAnalyzer = new StaticAnalyzer(model, algebraicModel/*, solver*/, provider, childAnalyzer);

			//// Request output
			//childAnalyzer.Logs[subdomainID] = new LinearAnalyzerLogFactory(new int[] { 0 });

			//var watchDofs = new Dictionary<int, int[]>();
			//watchDofs.Add(subdomainID, new int[] { 0, 1 });
			//var log1 = new TotalLoadsDisplacementsPerIncrementLog(model.NodesDictionary[142], StructuralDof.TranslationZ, (IEnumerable<INodalBoundaryCondition>)nodalBoundaryCondition, (IVectorValueExtractor)solution,   " ");
			//var log1 = new TotalLoadsDisplacementsPerIncrementLog(model.SubdomainsDictionary[subdomainID], 2, model.NodesDictionary[142], StructuralDof.TranslationZ, " ");
			//childAnalyzer.IncrementalLog = log1;
			
			// Run the analysis
			parentAnalyzer.Initialize();
			//var forces = model.SubdomainsDictionary.ElementAt(0).Value.GetRhsFromSolution(solution, dSolution);

			//var forces = subdomainUpdaters[0].CalculateResponseIntegralVector(solution);

			//double totalForce = forces.CopyToArray().Sum();
			//double expectedTotalForce = 14.04535167391627;
			//Assert.True(((totalForce - expectedTotalForce) / expectedTotalForce) < 1e-8);
			Assert.True(1 > 2);
		}



	}
}
