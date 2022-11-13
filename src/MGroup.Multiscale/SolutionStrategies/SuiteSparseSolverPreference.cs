using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using MGroup.LinearAlgebra.Matrices;
using MGroup.MSolve.Discretization.Entities;
using MGroup.MSolve.Solution;
using MGroup.Solvers.AlgebraicModel;
using MGroup.Solvers.Direct;
using MGroup.Solvers.DofOrdering;
using MGroup.Solvers.DofOrdering.Reordering;
using MGroup.LinearAlgebra.Matrices;
using MGroup.MSolve.Discretization.Entities;
using MGroup.MSolve.Solution;
using MGroup.Solvers.AlgebraicModel;

namespace MiMsolve.SolutionStrategies
{
    public class SuiteSparseSolverPreference : IAlgebraicStrategy<SymmetricCscMatrix>
    {
        
        public (ISolver, GlobalAlgebraicModel<SymmetricCscMatrix>) GetSolver(Model model)
        {
            var solverFactory = new SuiteSparseSolver.Factory();
            solverFactory.DofOrderer = new DofOrderer(new NodeMajorDofOrderingStrategy(), new NullReordering());
            var globalAlgebraicModel = solverFactory.BuildAlgebraicModel(model);
            var solver = solverFactory.BuildSolver(globalAlgebraicModel);
            return (solver, globalAlgebraicModel);
        }

        //GlobalAlgebraicModel<SkylineMatrix> IAlgebraicStrategy<SkylineMatrix>.GetAlgebraicModel(Model model)
        //{
        //    throw new NotImplementedException();
        //}

        //ISolver IAlgebraicStrategy<SkylineMatrix>.GetSolver(GlobalAlgebraicModel<SkylineMatrix> algebraicModel)
        //{
        //    throw new NotImplementedException();
        //}
    }

    //public class SkylineSolverPrefernce2<TMatrix> : IAlgebraicStrategy<TMatrix> where TMatrix : class, IMatrix
    //{
    //    public GlobalAlgebraicModel<TMatrix> GetAlgebraicModel(Model model)
    //    {
    //        throw new NotImplementedException();
    //    }

    //    public ISolver GetSolver(GlobalAlgebraicModel<TMatrix> algebraicModel)
    //    {
    //        throw new NotImplementedException();
    //    }
    //}

}
