namespace MGroup.Multiscale.Tests.RveTemplates.Tests.HomogeneousRVEBuilderNonLinearTest
{
	using System;
	using System.Collections.Generic;
	using System.Text;
	using MGroup.Constitutive.Structural.BoundaryConditions;
	using MGroup.Constitutive.Structural;
	using MGroup.MSolve.Discretization.Dofs;
	using MGroup.MSolve.Discretization.Entities;
	using MGroup.MSolve.MultiscaleAnalysis;
	using MGroup.NumericalAnalyzers.Discretization.NonLinear;
	using MGroup.NumericalAnalyzers;
	using MGroup.Solvers.Direct;

	using Xunit;
	using MGroup.MSolve.MultiscaleAnalysis.SupportiveClasses;
	using MGroup.Multiscale.Tests.FEMpartB;

	public class HomogeneousRVEBuilderNonLinearTest
	{
		[Fact]
		public static void CheckHomogeneousRVEBuilderNonLinearExample()
		{
			double[] computedDisplacements = SolveHomogeneousRVEBuilderNonLinearExample();
			string resultsPath = "..\\..\\RveTemplates\\Input\\ExpectedResults\\HomogeneousRVEBuilderNonLinearResults.txt";
			double[] expectedDisplacements = PrintUtilities.ReadVector(resultsPath);

			bool check = ComparisonMetods.AreDisplacementsSame(expectedDisplacements, computedDisplacements);

			Assert.True(check);
		}

		public static double[] SolveHomogeneousRVEBuilderNonLinearExample()
		{
			var rve = new HomogeneousRVEBuilderNonLinear();
			var tuple1 = rve.GetModelAndBoundaryNodes();
			Model model = tuple1.Item1;

			model.BoundaryConditions.Add(new StructuralBoundaryConditionSet(
				new[]
				{
					new NodalDisplacement(model.NodesDictionary[1], StructuralDof.TranslationX, amount: 0),
					new NodalDisplacement(model.NodesDictionary[1], StructuralDof.TranslationY, amount: 0),
					new NodalDisplacement(model.NodesDictionary[1], StructuralDof.TranslationZ, amount: 0),
					new NodalDisplacement(model.NodesDictionary[37], StructuralDof.TranslationX, amount: 0),
					new NodalDisplacement(model.NodesDictionary[37], StructuralDof.TranslationY, amount: 0),
					new NodalDisplacement(model.NodesDictionary[37], StructuralDof.TranslationZ, amount: 0),
					new NodalDisplacement(model.NodesDictionary[57], StructuralDof.TranslationX, amount: 0),
					new NodalDisplacement(model.NodesDictionary[57], StructuralDof.TranslationY, amount: 0),
					new NodalDisplacement(model.NodesDictionary[57], StructuralDof.TranslationZ, amount: 0)
				},
				new[]
				{
					new NodalLoad(model.NodesDictionary[10], StructuralDof.TranslationY, amount: -10d),
					new NodalLoad(model.NodesDictionary[10], StructuralDof.TranslationX, amount: 5d),
					new NodalLoad(model.NodesDictionary[10], StructuralDof.TranslationZ, amount: 10d)
				}));

			var solverFactory = new SkylineSolver.Factory();
			var algebraicModel = solverFactory.BuildAlgebraicModel(model);
			var solver = solverFactory.BuildSolver(algebraicModel);

			var problem = new ProblemStructural(model, algebraicModel, solver);
			var loadControlAnalyzerBuilder = new LoadControlAnalyzer.Builder(model, algebraicModel, solver, problem, numIncrements: 2)
			{
				ResidualTolerance = 1E-3,
				MaxIterationsPerIncrement = 1000,
				NumIterationsForMatrixRebuild = 2
			};
			var loadControlAnalyzer = loadControlAnalyzerBuilder.Build();

			var staticAnalyzer = new StaticAnalyzer(model, algebraicModel/*, solver*/, problem, loadControlAnalyzer);

			staticAnalyzer.Initialize();
			staticAnalyzer.Solve();
			var solution = solver.LinearSystem.Solution.SingleVector;

			return solution.CopyToArray();
		}
	}
}
