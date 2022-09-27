using System;
using System.Collections.Generic;
using MGroup.LinearAlgebra.Vectors;
using MGroup.MSolve.AnalysisWorkflow;
using MGroup.MSolve.AnalysisWorkflow.Logging;
using MGroup.MSolve.AnalysisWorkflow.Providers;
using MGroup.MSolve.DataStructures;
using MGroup.MSolve.Discretization;
using MGroup.MSolve.Discretization.Entities;
using MGroup.MSolve.Solution;
using MGroup.MSolve.Solution.AlgebraicModel;
using MGroup.MSolve.Solution.LinearSystem;

namespace MiMsolve.multiScaleSupportiveClasses
{
    /// <summary>
    /// Parent Analyzer for nonlinear static problems assosiated with a microstructure bvp handled by MicrostructureBvpNRNLAnalyzer
    /// Authors: Gerasimos Sotiropoulos
    /// </summary>
    public class MSParentAnalyzer : INonLinearParentAnalyzer
    {
        //private readonly IReadOnlyDictionary<int, ILinearSystem> linearSystems;
        private readonly IModel model;
        private readonly INonTransientAnalysisProvider provider;
        private readonly ISolver solver;
        private bool firstInitialization;
        private readonly IAlgebraicModel algebraicModel;


        public MSParentAnalyzer(IModel model,  IAlgebraicModel algebraicModel, ISolver solver, INonTransientAnalysisProvider provider, 
            IChildAnalyzer childAnalyzer)
        {
            this.model = model;
            this.algebraicModel = algebraicModel;
            this.solver = solver;
            this.provider = provider;
            this.ChildAnalyzer = childAnalyzer;
            this.ChildAnalyzer.ParentAnalyzer = this;
        }

        public IAnalysisWorkflowLog[] Logs { get; set; }

        public IChildAnalyzer ChildAnalyzer { get; }
		public GenericAnalyzerState CurrentState { get => throw new NotImplementedException(); set => throw new NotImplementedException(); }

		public void BuildMatrices()
        {
            provider.CalculateMatrix();

        }

        public IGlobalVector GetOtherRhsComponents(IGlobalVector currentSolution)
        {
            //TODO: use a ZeroVector class that avoid doing useless operations or refactor this method. E.g. let this method 
            // alter the child analyzer's rhs vector, instead of the opposite (which is currently done).
            return algebraicModel.CreateZeroVector();
        }

        public void Initialize(bool isFirstAnalysis = true)
        {

            if (ChildAnalyzer == null) throw new InvalidOperationException("Static analyzer must contain an embedded analyzer.");
            
            
            //foreach (ILinearSystem linearSystem in linearSystems.Values)
            //{
            //    linearSystem.RhsVector = linearSystem.Subdomain.Forces;
            //}
            //TODO:Ger subdomain.Forces doesn't exist anymore.

            ChildAnalyzer.Initialize(isFirstAnalysis);
        }

        public void Solve()
        {
            if (ChildAnalyzer == null) throw new InvalidOperationException("Static analyzer must contain an embedded analyzer.");
            BuildMatrices(); //TODO: this should be called by the class that calls model.AssignLoads() and before it. 
            ChildAnalyzer.Solve();
        }

		public GenericAnalyzerState CreateState() => throw new NotImplementedException();
		IHaveState ICreateState.CreateState() => throw new NotImplementedException();
	}
}
