namespace MGroup.Multiscale.Tests.RveTemplates.Tests.RveGrShOneTest
{
	using System;
	using System.Collections.Generic;
	using System.Text;

	using MGroup.Constitutive.Structural;
	using MGroup.Constitutive.Structural.BoundaryConditions;
	using MGroup.LinearAlgebra.Commons;
	using MGroup.LinearAlgebra.Vectors;
	using MGroup.MSolve.Discretization.Dofs;
	using MGroup.MSolve.Discretization.Entities;
	using MGroup.MSolve.MultiscaleAnalysis.SupportiveClasses;
	using MGroup.MSolve.Solution.LinearSystem;
	using MGroup.Multiscale.RveTemplates;
	using MGroup.Multiscale.Tests.FEMpartB;
	using MGroup.NumericalAnalyzers;
	using MGroup.NumericalAnalyzers.Logging;
	using MGroup.Solvers.Direct;

	using Xunit;
	public class RveGrShOneTest
	{
		[Fact]
		public static void CheckRveGrShOneExample()
		{
			double[] computedDisplacements = SolveRveGrShOneExample();
			string resultsPath = "..\\..\\RveTemplates\\Input\\ExpectedResults\\RveGrShOneResults.txt";
			double[] expectedDisplacements = PrintUtilities.ReadVector(resultsPath);

			bool check = AreDisplacementsSame(expectedDisplacements, computedDisplacements);
			Assert.True(check);
		}

		public static double[] SolveRveGrShOneExample()
		{
			var rve = new RveGrShOne(0);
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
					new NodalDisplacement(model.NodesDictionary[57], StructuralDof.TranslationZ, amount: 0),
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
			var linearAnalyzer = new LinearAnalyzer(algebraicModel, solver, problem);
			var staticAnalyzer = new StaticAnalyzer(model, algebraicModel/*, solver*/, problem, linearAnalyzer);

			staticAnalyzer.Initialize();
			staticAnalyzer.Solve();
			var solution = solver.LinearSystem.Solution.SingleVector;

			return solution.CopyToArray();
		}

		public static bool AreDisplacementsSame(double[] expectedValues,
			double[] computedValues, double tol = 1E-9)
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
	}
}
