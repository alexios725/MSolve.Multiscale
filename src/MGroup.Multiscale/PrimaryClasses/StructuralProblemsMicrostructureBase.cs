using MGroup.Constitutive.Structural;
using MGroup.Constitutive.Structural.Providers;
using MGroup.MSolve.Discretization;
using MGroup.MSolve.Discretization.Dofs;
using MGroup.MSolve.Discretization.Entities;
using MGroup.MSolve.Solution;
using MGroup.MSolve.Solution.AlgebraicModel;
using MGroup.MSolve.Solution.LinearSystem;
using MGroup.NumericalAnalyzers.Discretization.NonLinear;
using MiMsolve.multiScaleSupportiveClasses;
using System.Collections.Generic;
using System.Linq;

namespace MGroup.MSolve.MultiscaleAnalysis
{
    /// <summary>
    /// base class for Primary multiscale analysis classes that connect nesessary structures for a FE2 simulation
    /// Authors: Gerasimos Sotiropoulos
    /// </summary>
    public abstract class StructuralProblemsMicrostructureBase
    {
		internal IGlobalVector PeviousResidual {  get;  set; }
		public int SolverData { get; set; }

		
		public virtual Dictionary<int, IElementType> GetBoundaryFiniteElementsDictionaryOfSingleSubdomainModel(IModel model, Dictionary<int, Node> boundaryNodes)
        {
            Dictionary<int, IElementType> boundaryElements = new Dictionary<int, IElementType>();

            foreach (var element in model.EnumerateElements(model.EnumerateSubdomains().First().ID))
            {
                bool containsBoundaryNode = false;

                var elementDOFTypes = element.DofEnumerator.GetDofTypesForMatrixAssembly(element);
                var matrixAssemblyNodes = element.DofEnumerator.GetNodesForMatrixAssembly(element);
                for (int j = 0; j < elementDOFTypes.Count; j++)
                {
                    INode elementNode = matrixAssemblyNodes[j];
                    if (boundaryNodes.ContainsKey(elementNode.ID))
                    {
                        containsBoundaryNode = true;
                        break;
                    }
                }

                if (containsBoundaryNode)
                {
                    boundaryElements.Add(element.ID, element);
                }

            }

            return boundaryElements;

        }

        public virtual Dictionary<int, IElementType> GetBoundaryFiniteElementsDictionary(Subdomain subdomain, Dictionary<int, INode> boundaryNodes)
        {
            Dictionary<int, IElementType> subdomainBoundaryElements = new Dictionary<int, IElementType>();

            foreach (IElementType element in subdomain.EnumerateElements())
            {
                bool containsBoundaryNode = false;

                var elementDOFTypes = element.DofEnumerator.GetDofTypesForMatrixAssembly(element);
                var matrixAssemblyNodes = element.DofEnumerator.GetNodesForMatrixAssembly(element);
                for (int j = 0; j < elementDOFTypes.Count; j++)
                {
                    INode elementNode = matrixAssemblyNodes[j];
                    if (boundaryNodes.ContainsKey(elementNode.ID))
                    {
                        containsBoundaryNode = true;
                        break;
                    }
                }

                if (containsBoundaryNode)
                {
                    subdomainBoundaryElements.Add(element.ID, element);
                }

            }

            return subdomainBoundaryElements;

        }

        //TODO Ger: Do not erase this. It is for the multisubdomain implementation
        //public virtual Dictionary<int, Dictionary<int, IElementType>> GetSubdomainsBoundaryFiniteElementsDictionaries(Model model, Dictionary<int, Node> boundaryNodes)
        //{
        //    Dictionary<int, Dictionary<int, IElementType>> subdomainsBoundaryElements = new Dictionary<int, Dictionary<int, IElementType>>();

        //    foreach (Subdomain subdomain in model.SubdomainsDictionary.Values)
        //    {
        //        Dictionary<int, IElementType> subdBoundaryElements = GetBoundaryFiniteElementsDictionary(subdomain, boundaryNodes);
        //        subdomainsBoundaryElements.Add(subdomain.ID, subdBoundaryElements);
        //    }

        //    return subdomainsBoundaryElements;

        //}

        public virtual Dictionary<int, IElementType> GetSubdomainsBoundaryFiniteElementsDictionaries(Model model, Dictionary<int, INode> boundaryNodes)
        {
            Dictionary<int, IElementType> subdBoundaryElements = GetBoundaryFiniteElementsDictionary(model.SubdomainsDictionary.ElementAt(0).Value, boundaryNodes);
            return subdBoundaryElements;
        }

        public virtual (DisplacementBvpNRNLAnalyzer, ElementStructuralStiffnessProvider) AnalyzeMicrostructure(Model model,  ISolver solver,
            int increments, int MaxIterations, int IterationsForMatrixRebuild, Dictionary<int, Dictionary<IDofType, double>> totalPrescribedBoundaryDisplacements,
            Dictionary<int, Dictionary<IDofType, double>> initialConvergedBoundaryDisplacements, Dictionary<int, INode> boundaryNodes,
            IGlobalVector uInitialFreeDOFDisplacementsPerSubdomain, IAlgebraicModel algebraicModel , ProblemStructural provider)
        {
            //IReadOnlyDictionary<int, ILinearSystem> linearSystems = solver.LinearSystems; //V2.1

            #region Creation of nessesary analyzers for NRNLAnalyzer
            //ProblemStructural provider = new ProblemStructural(model, algebraicModel, solver);

            var subdomainUpdaters = new NonLinearModelUpdaterWithInitialConditions(algebraicModel);

            ElementStructuralStiffnessProvider elementProvider = new ElementStructuralStiffnessProvider();

            //v2.4
            //Dictionary<int, EquivalentContributionsAssebler> equivalentContributionsAssemblers = new Dictionary<int, EquivalentContributionsAssebler>();//SUNOLIKA STOIXEIA model.SubdomainsDictionary.Count oi oles tis model.subdomains ekei mallon deginontai access me ID.
            ////equivalentContributionsAssemblers.Add(model.SubdomainsDictionary[1].ID, new EquivalentContributionsAssebler(model.SubdomainsDictionary[1], elementProvider));
            //foreach (Subdomain subdomain in model.SubdomainsDictionary.Values)
            //{
            //    equivalentContributionsAssemblers.Add(subdomain.ID, new EquivalentContributionsAssebler(subdomain, elementProvider)); //v2.5
            //}
            #endregion

            #region Creation of Microstructure analyzer (NRNLdevelop temporarilly). 
            DisplacementBvpNRNLAnalyzer microAnalyzer = new DisplacementBvpNRNLAnalyzer(model,solver, subdomainUpdaters, 
                provider, increments,  uInitialFreeDOFDisplacementsPerSubdomain,
                boundaryNodes, initialConvergedBoundaryDisplacements, totalPrescribedBoundaryDisplacements, algebraicModel);
            microAnalyzer.SetMaxIterations = MaxIterations;
            microAnalyzer.SetIterationsForMatrixRebuild = IterationsForMatrixRebuild;
            #endregion

            #region solution and update ------------->THA MPEI ENTOS KLASHS: of free converged displacements vectors;
            MSParentAnalyzer parentAnalyzer = new MSParentAnalyzer(model, algebraicModel, solver, provider, microAnalyzer);
             //parentAnalyzer.BuildMatrices(); //v2.6 ston neon static analyzer den xreiazetai to build matrices poia
            parentAnalyzer.Initialize();
            parentAnalyzer.Solve();
            #endregion

            return (microAnalyzer,elementProvider);
        }
    }
}
