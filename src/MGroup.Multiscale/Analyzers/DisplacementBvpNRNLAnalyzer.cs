using System;
using System.Collections.Generic;
using System.Diagnostics;
using System;
using System.Diagnostics;
using MGroup.MSolve.AnalysisWorkflow;
using MGroup.MSolve.AnalysisWorkflow.Providers;
using MGroup.NumericalAnalyzers.Logging;
using MGroup.MSolve.Discretization;
using MGroup.MSolve.Discretization.Entities;
using MGroup.MSolve.Solution;
using MGroup.NumericalAnalyzers.Discretization.NonLinear;
using MGroup.MSolve.Solution.AlgebraicModel;
using MGroup.MSolve.Solution.LinearSystem;
using MGroup.LinearAlgebra.Vectors;
using MGroup.MSolve.Discretization.Dofs;
using MGroup.NumericalAnalyzers.Logging;
using MGroup.MSolve.AnalysisWorkflow.Logging;
using MiMsolve.intermediateCodeDevelopmentClasses;
using MGroup.Constitutive.Structural.Providers;
using System.Linq;
using MGroup.MSolve.DataStructures;

namespace MiMsolve.multiScaleSupportiveClasses
{
    /// <summary>
    /// Newton Raphson Non-Linear Analyzer for the solution of boundary value problems that arise 
    /// from a FE2 multiscale Analysis and are related to the microstructure model.
    /// Authors: Gerasimos Sotiropoulos
    /// </summary>
    public class DisplacementBvpNRNLAnalyzer : IChildAnalyzer
    {
        protected readonly IAlgebraicModel algebraicModel;
        protected readonly IModel model;
        private readonly NonLinearModelUpdaterWithInitialConditions subdomainUpdaters;
        //private readonly ISubdomainGlobalMapping[] mappings; 
        private readonly int increments;
        //private readonly int totalDOFs;
        private int maxSteps = 1000;
        private int stepsForMatrixRebuild = 0;
        private readonly double tolerance = 1e-3;
        private double rhsNorm;
        private INonLinearParentAnalyzer parentAnalyzer = null;
        private readonly ISolver solver;
        private readonly INonLinearProvider provider;
        private readonly IGlobalVector rhs ;//comment MS2:apothikevetai se afto h timh (externalLoads/increments) gia kathe subdomain kai apo ekei pernietai opou xreiasthei (p.x. subdomain.RHS)
        private IGlobalVector u ;
        private IGlobalVector du ;
        private IGlobalVector uPlusdu ;
        Dictionary<int, INode> boundaryNodes;
        Dictionary<int, Dictionary<IDofType, double>> initialConvergedBoundaryDisplacements;
        Dictionary<int, Dictionary<IDofType, double>> totalBoundaryDisplacements;
        //private readonly ElementEquivalentContributionsProvider equivalentContributionsAssemblers;
        private IGlobalVector globalRhs;
        private readonly Dictionary<int, LinearAnalyzerLogFactory> logFactories = new Dictionary<int, LinearAnalyzerLogFactory>();
        private readonly Dictionary<int, IAnalysisWorkflowLog[]> logs = new Dictionary<int, IAnalysisWorkflowLog[]>();

        #region necessary logs for dispalcements and forces
        public Dictionary<int, TotalLoadsDisplacementsPerIncrementLog> IncrementalLogs { get; }
            = new Dictionary<int, TotalLoadsDisplacementsPerIncrementLog>();

        //public IReactionsLog ForcesLog { get; set; } 

        #endregion

        public DisplacementBvpNRNLAnalyzer(IModel model, ISolver solver, NonLinearModelUpdaterWithInitialConditions subdomainUpdaters,
            INonLinearProvider provider, int increments, IGlobalVector uInitialFreeDOFDisplacementsPerSubdomain,
            Dictionary<int, INode> boundaryNodes, Dictionary<int, Dictionary<IDofType, double>> initialConvergedBoundaryDisplacements, Dictionary<int, Dictionary<IDofType, double>> totalBoundaryDisplacements,
           /*ElementEquivalentContributionsProvider equivalentContributionsAssemblers,*/ IAlgebraicModel algebraicModel)//, ISubdomainGlobalMapping[] mappings)
        {
            this.model = model;
            this.solver = solver;
            this.subdomainUpdaters = subdomainUpdaters;
            //this.mappings = mappings;
            this.algebraicModel = algebraicModel;
            //linearSystems = solver.LinearSystems;
            this.provider = provider;
            this.increments = increments;
            //this.globalRhs = Vector.CreateZero(model.GlobalDofOrdering.NumGlobalFreeDofs); 
            u = uInitialFreeDOFDisplacementsPerSubdomain; //prosthiki MS, TODO:possibly check for compatibility elements format: u.Add(subdomain.ID, new Vector(subdomain.RHS.Length));
                                                               //this.uPlusdu = uInitialFreeDOFDisplacementsPerSubdomain; //prosthiki MS: commented out possible pass by reference
            this.boundaryNodes = boundaryNodes;
            this.initialConvergedBoundaryDisplacements = initialConvergedBoundaryDisplacements;
            this.totalBoundaryDisplacements = totalBoundaryDisplacements;
            //this.equivalentContributionsAssemblers = equivalentContributionsAssemblers;

            rhs = algebraicModel.CreateZeroVector();


            //InitializeInternalVectors();
        }

        public int SetMaxIterations
        {
            set
            {
                if (value > 0) { maxSteps = value; }
                else { throw new Exception("Iterations number cannot be negative or zero"); }
            }
        }


        public int SetIterationsForMatrixRebuild
        {
            set
            {
                if (value > 0) { stepsForMatrixRebuild = value; }
                else { throw new Exception("Iterations number for matrix rebuild cannot be negative or zero"); }
            }
        }


        private void InitializeLogs()
        {
            logs.Clear();
            foreach (int id in logFactories.Keys) logs.Add(id, logFactories[id].CreateLogs());
        }

        //private void StoreLogResults(DateTime start, DateTime end)
        //{
        //    foreach (int id in logs.Keys)
        //        foreach (var l in logs[id])
        //            l.StoreResults(start, end, linearSystems[id].Solution);
        //}

        public Dictionary<int, LinearAnalyzerLogFactory> LogFactories { get { return logFactories; } }

        #region IAnalyzer Members

        //public Dictionary<int, IAnalysisWorkflowLog[]> Logs { get { return logs; } }
        public IAnalysisWorkflowLog[] Logs { get; set; } = new IAnalysisWorkflowLog[0];

        public IParentAnalyzer ParentAnalyzer// exei diorthothei apo bas v2
        {
            get => parentAnalyzer;
            set => parentAnalyzer = (INonLinearParentAnalyzer)value; //TODO: remove this cast. Now it only serves as a check
        }

        public IAnalyzer ChildAnalyzer
        {
            get { return null; }
            set { throw new InvalidOperationException("Newton-Raphson analyzer cannot contain an embedded analyzer."); }
        }

		private GenericAnalyzerState currentState;
		public GenericAnalyzerState CurrentState { get => currentState; set => throw new NotImplementedException(); }

		public IGlobalVector CurrentAnalysisLinearSystemRhs => throw new NotImplementedException();

		public IGlobalVector CurrentAnalysisResult => throw new NotImplementedException();

		public void InitializeInternalVectors()//TODOMaria: this is probably where the initial internal nodal vector is calculated
        {
            globalRhs = algebraicModel.CreateZeroVector(); 
            rhs.Clear();
            rhs.CopyFrom(solver.LinearSystem.RhsVector);
            rhs.ScaleIntoThis(1 / (double)increments);
            //u.Clear(); prosthiki MS
            if (du is null)
            {
                du = algebraicModel.CreateZeroVector();
                uPlusdu = algebraicModel.CreateZeroVector();
                uPlusdu.AddIntoThis(u);
            }
            else
            {
                du.Clear();
                uPlusdu.Clear(); //prosthiki MS
                uPlusdu.AddIntoThis(u);
            }
            //TODO: globalRhs.AddIntoThis()
            //gia thn edw 165 opote einai to idio me me thn %136 tou NonlinearAnalyzerBAse
            globalRhs.AddIntoThis(solver.LinearSystem.RhsVector);
            //foreach (ILinearSystem linearSystem in linearSystems.Values)
            //{
            //    int id = linearSystem.Subdomain.ID;

            //    //IVector r = linearSystem.RhsVector.Copy();
            //    //r.ScaleIntoThis(1 / (double)increments);
            //    //rhs.Add(id, r); //comment MS: xtizei to rhs, field tou NRNLAnalyzer ok, swsta giati rhs sth dhmiourgia twn linear systems exoume perasei to model.Subdomains[0].Forces
            //                    //u.Add(subdomain.ID, new Vector(subdomain.RHS.Length)); //prosthiki MS
            //    //du.Add(id, linearSystem.CreateZeroVector());
            //    //var tempcopy = u[id].CopyToArray();
            //    //uPlusdu.Add(id, Vector.CreateFromArray(tempcopy));
            //    model.GlobalDofOrdering.AddVectorSubdomainToGlobal(linearSystem.Subdomain, linearSystem.RhsVector, globalRhs);

            //    //Vector tempCopy = new Vector(u[linearSystem.ID].Length);
            //    //u[linearSystem.ID].Data.CopyTo(tempCopy.Data, 0);
            //    //uPlusdu.Add(linearSystem.ID, tempCopy); //prosthiki MS
            //    //mappings[linearSystems.Select((v, i) => new { System = v, Index = i }).First(x => x.System.ID == linearSystem.ID).Index].SubdomainToGlobalVector(((Vector)linearSystem.RHS).Data, globalRhs.Data);
            //}


            rhsNorm = provider.CalculateRhsNorm(globalRhs);
        }

        public void InitializeLinearSystemRhsVector()
        {
            IGlobalVector forces = algebraicModel.CreateZeroVector();
            algebraicModel.LinearSystem.RhsVector = forces;
        }

        private void UpdateInternalVectors()//TODOMaria this is where I should add the calculation of the internal nodal force vector
        {
            globalRhs.Clear();
            rhs.Clear();
            rhs.CopyFrom(solver.LinearSystem.RhsVector);
            rhs.ScaleIntoThis(1 / (double)increments);
            globalRhs.AddIntoThis(solver.LinearSystem.RhsVector);

            //foreach (ILinearSystem linearSystem in linearSystems.Values)
            //{
            //    int id = linearSystem.Subdomain.ID;

            //    IVector r = linearSystem.RhsVector.Copy();
            //    r.ScaleIntoThis(1 / (double)increments);
            //    rhs[id] = r; //comment MS: xanahtizei to rhs kai to xanaperna sto globalRHS(molis to kane clear opote ok)
            //    model.GlobalDofOrdering.AddVectorSubdomainToGlobal(linearSystem.Subdomain, linearSystem.RhsVector, globalRhs);
            //}
            rhsNorm = provider.CalculateRhsNorm(globalRhs);
        }

        public void Initialize(bool isFirstAnalysis = true)
        {
            InitializeInternalVectors();
            //solver.Initialize(); //TODO: Using this needs refactoring
        }

        private void UpdateRHS(int step)
        {
            solver.LinearSystem.RhsVector.CopyFrom(rhs);
            //foreach (ILinearSystem linearSystem in linearSystems.Values)
            //{
            //    linearSystem.RhsVector.CopyFrom(rhs[linearSystem.Subdomain.ID]);
            //}
        }

        public void Solve()
        {
            InitializeLogs();

            DateTime start = DateTime.Now;
            UpdateInternalVectors();//TODOMaria this divides the externally applied load by the number of increments and scatters it to all subdomains and stores it in the class subdomain dictionary and total external load vector
            for (int increment = 0; increment < increments; increment++)
            {
                //CnstValuesNLShell.macroNRIncrement = increment; // (recent cnst code output)

                #region solver parameters
                //CnstValues.analyzerLoadingStep = increment;
                #endregion
                double errorNorm = 0;
                ClearIncrementalSolutionVector();//TODOMaria this sets du to 0
                UpdateRHS(increment);//comment MS2: apo to rhs[subdomain.ID] pernaei sto subdomain.RHS h fixed timh (externalLoads/increments) (ginetai copy kai oxi add)  AFTO thewreitai RHS sthn prwth iteration
                UpdateRHSForLinearizationContributions(increment + 1, increments);// opote edw pouu uparxei to neo rhs tou kuklou epanalhpsewn prepei na afairountai oi draseis
                double firstError = 0;
                int step = 0;
                for (step = 0; step < maxSteps; step++)
                {
                    #region shell fe2 health monitoring (recent cnst code output)
                    //CnstValuesNLShell.macroNRIter = step;
                    //if (CnstValuesNLShell.WriteOutput)
                    //{
                    //    //(new ISAAR.MSolve.LinearAlgebra.Output.Array2DWriter()).WriteToFile(solver.LinearSystems[0].Matrix.CopytoArray2D(), CnstValuesNLShell.totalStiffnessStringIterInfo);
                    //    new ISAAR.MSolve.LinearAlgebra.Output.Array1DWriter().WriteToFile(new double[] { solver.LinearSystems[0].RhsVector.Length }, CnstValuesNLShell.globalStiffnessLengthIterInfo);
                    //    new ISAAR.MSolve.LinearAlgebra.Output.Array1DWriter().WriteToFile(solver.LinearSystems[0].RhsVector.CopyToArray(), CnstValuesNLShell.totalRHsStringIterInfo);
                    //}
                    #endregion
                    #region solver parameters
                    //CnstValues.analyzerNRIter += 1;
                    //CnstValues.analyzerInfo = "Solution";
                    //CnstValues.analyzerInfoIsSolutionForNRiters = true;
                    #endregion
                    solver.Solve();
                    errorNorm = rhsNorm != 0 ? CalculateInternalRHS(increment, step, increments) / rhsNorm : 0;//comment MS2: to subdomain.RHS lamvanei thn timh nIncrement*(externalLoads/increments)-interanalRHS me xrhsh ths fixed timhs apo to rhs[subdomain.ID]
                    Debug.WriteLine("INFO NR step {0}, iteration {1}, norm: {2}", increment, step, errorNorm);
                    if (step == 0) firstError = errorNorm;
                    if (errorNorm < tolerance)
                    {
                        //CnstValues.analyzerInfo = "Homogenization"; // (recent cnst code output)
                        //CnstValues.analyzerInfoIsSolutionForNRiters = false;

                        int subdomainID = 0;
                        int c1 = 0;
                        foreach (var subdomainLogPair in IncrementalLogs)
                        {
                            if (c1 == 0) { subdomainID = subdomainLogPair.Key; }
                            TotalLoadsDisplacementsPerIncrementLog log = subdomainLogPair.Value;
                            log.LogTotalDataForIncrement(increment, step, errorNorm,
                                uPlusdu, globalRhs);
                            c1++;
                        }
                        //ForcesLog.CalculateAndStoreSubdomainReactionAndForces((Model)model, boundaryNodes,
                        //     u[ForcesLog.subdId], du[ForcesLog.subdId],
                        //     initialConvergedBoundaryDisplacements, totalBoundaryDisplacements,
                        //     increment + 1, increments); //TODO Ger

                        break;
                    }

                    SplitResidualForcesToSubdomains();//TODOMaria scatter residuals to subdomains
                    if ((step + 1) % stepsForMatrixRebuild == 0)
                    {
                        provider.Reset();
                        BuildMatrices();
                        //solver.Initialize(); //TODO: Using this needs refactoring
                    }
                }
                Debug.WriteLine("NR {0}, first error: {1}, exit error: {2}", step, firstError, errorNorm);
                UpdateSolution();
                SaveMaterialState();
            }
            CopySolutionToSubdomains();//TODOMaria Copy current displacement to subdomains
                                       //            ClearMaterialStresses();
            DateTime end = DateTime.Now;

            //StoreLogResults(start, end);
        }

        private double CalculateInternalRHS(int currentIncrement, int step, int totalIncrements)
        {
            globalRhs.Clear();
            //var isNodeUpdated = new BooleanArray(model.NumNodes + 1); var areBoundaryNodesUpdated = new BooleanArray(boundaryNodes.Count + 1);

            if (currentIncrement == 0 && step == 0)
            {
                du.AddIntoThis(solver.LinearSystem.Solution);
                uPlusdu.Clear();
                uPlusdu.AddIntoThis(u);
                uPlusdu.AddIntoThis(du);
            }
            else
            {
                du.AddIntoThis(solver.LinearSystem.Solution);
                uPlusdu.Clear();
                uPlusdu.AddIntoThis(u);
                uPlusdu.AddIntoThis(du);
            }

            var boundaryNodesIds = boundaryNodes.Keys.ToList();
            var internalRhs = subdomainUpdaters.GetRHSFromSolutionWithInitialDisplacemntsEffect(uPlusdu, boundaryNodes,
             initialConvergedBoundaryDisplacements, totalBoundaryDisplacements, currentIncrement+1, totalIncrements);

            provider.ProcessInternalRhs(uPlusdu, internalRhs);// this does nothing

            if (parentAnalyzer != null)
            {
                var otherRhsComponents = parentAnalyzer.GetOtherRhsComponents(uPlusdu);
                internalRhs.AddIntoThis(otherRhsComponents);// this does nothing for the static problem
            }//TODOMaria this does nothing for the static problem


            solver.LinearSystem.RhsVector.Clear();
            for (int j = 0; j <= currentIncrement; j++) solver.LinearSystem.RhsVector.AddIntoThis(rhs);//TODOMaria this adds the external forces 
            solver.LinearSystem.RhsVector.SubtractIntoThis(internalRhs);
            globalRhs.AddIntoThis(solver.LinearSystem.RhsVector);
           
            double providerRHSNorm = provider.CalculateRhsNorm(globalRhs);
            return providerRHSNorm;
        }

        private void ClearIncrementalSolutionVector()
        {
            du.Clear();
            
        }

        private void SplitResidualForcesToSubdomains()
        {
            solver.LinearSystem.RhsVector.Clear();
            solver.LinearSystem.RhsVector.CopyFrom(globalRhs);
                       
        }

        private void UpdateSolution()
        {
            u.AddIntoThis(du);
        }//.

        public void SaveMaterialState()
        {
            subdomainUpdaters.UpdateState(currentState);
        }

        public IGlobalVector GetConvergedSolutionVectorsOfFreeDofs()
        {
            return u;
            // return uplusdu einai to idio afou exei ginei molis UpdateSolution
        }

        public IGlobalVector GetConvergedIncrementalSolutionVectorsOfFreeDofs()
        {
            return du;
        }


        private void CopySolutionToSubdomains()
        {
                //TODO:
                //int id = linearSystem.Subdomain.ID;
                //u[id].CopyTo(((Vector)linearSystem.Solution).Data, 0);           
        }

        private void ClearMaterialStresses()
        {
            //subdomainUpdaters.ResetState(); <--- se progoumenh ekdosh ginotan etsi
            
        }

        public void BuildMatrices()
        {
            if (parentAnalyzer == null)
                throw new InvalidOperationException("This Newton-Raphson non-linear analyzer has no parent.");

            parentAnalyzer.BuildMatrices();
            //solver.Initialize();
        }

        private void UpdateRHSForLinearizationContributions(int nIncrement, int increments)
        {
            globalRhs.Clear();

            
            var elementStiffnessProvider = new ElementStructuralStiffnessProvider();
            var elementContributionVecprovider = new ElementEquivalentContributionsProvider(algebraicModel, elementStiffnessProvider, boundaryNodes,
             initialConvergedBoundaryDisplacements, totalBoundaryDisplacements, nIncrement, increments);
            
            IGlobalVector conttribution = algebraicModel.CreateZeroVector();
            algebraicModel.AddToGlobalVector(conttribution, elementContributionVecprovider);

            solver.LinearSystem.RhsVector.SubtractIntoThis(conttribution);
            globalRhs.AddIntoThis(solver.LinearSystem.RhsVector);

            
            rhsNorm = provider.CalculateRhsNorm(globalRhs);
            //TODOMS: possibly it is not nessesary to update globalRHS (and of course not clear it before the loop) as it is not updated in 177-UpdateRHS(increment) either.
            // and it is used when it is recalculated in CalculateInternalRHS......
        }

		public GenericAnalyzerState CreateState() => throw new NotImplementedException();
		IHaveState ICreateState.CreateState() => throw new NotImplementedException();

		#endregion
	}
}
