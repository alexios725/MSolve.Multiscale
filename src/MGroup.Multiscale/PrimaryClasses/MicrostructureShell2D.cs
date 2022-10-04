using ISAAR.MSolve.MultiscaleAnalysis.SupportiveClasses;
using MGroup.Constitutive.Structural;
using MGroup.Constitutive.Structural.Continuum;
using MGroup.Constitutive.Structural.Providers;
using MGroup.Constitutive.Structural.Shells;
using MGroup.LinearAlgebra.Matrices;
using MGroup.LinearAlgebra.Vectors;
using MGroup.MSolve.Constitutive;
using MGroup.MSolve.DataStructures;
using MGroup.MSolve.Discretization;
using MGroup.MSolve.Discretization.Dofs;
using MGroup.MSolve.Discretization.Entities;
using MGroup.MSolve.MultiscaleAnalysis.Interfaces;
using MGroup.MSolve.Solution;
using MGroup.MSolve.Solution.LinearSystem;
using MGroup.NumericalAnalyzers.Discretization.NonLinear;
using MGroup.Solvers.AlgebraicModel;
using MGroup.Solvers.Direct;
using MGroup.Solvers.DofOrdering;
using MGroup.Solvers.DofOrdering.Reordering;
using MGroup.Solvers.Results;
using MiMsolve.multiScaleSupportiveClasses;
using MiMsolve.SolutionStrategies;
using System;
using System.Collections.Generic;
using System.Linq;

namespace MGroup.MSolve.MultiscaleAnalysis
{
	/// <summary>
	/// Primary multiscale analysis class that connects all nesessary structures for a FE2 simulation (with an 3D to 2D degenerate Rve)
	/// Authors: Gerasimos Sotiropoulos
	/// </summary>
	public class MicrostructureShell2D<TMatrix> : StructuralProblemsMicrostructureBase, IShellMaterial //A.1
		where TMatrix : class, IMatrix
    {
		public double[] NormalVectorV3 { get; set; }
		public double[] TangentVectorV1 { get; set; }
		public double[] TangentVectorV2 { get; set; }
		private ProblemStructural provider;
        private IAlgebraicStrategy<TMatrix> algebraicStrategy;
        public GlobalAlgebraicModel<TMatrix> globalAlgebraicModel { get; private set; }
        private ISolver solver;
        public Model model { get; private set; }
        //private readonly Dictionary<int, Node> nodesDictionary = new Dictionary<int, Node>();
        private Dictionary<int, INode> boundaryNodes { get; set; }
         Dictionary<int, IElementType> boundaryElements;
		private IdegenerateRVEbuilder rveBuilder;
		private bool EstimateOnlyLinearResponse;
        //private NewtonRaphsonNonLinearAnalyzer microAnalyzer;
        private double volume;
        public IGlobalVector uInitialFreeDOFDisplacementsPerSubdomain { get; private set; }
        Dictionary<int, Dictionary<IDofType, double>> initialConvergedBoundaryDisplacements;
		private IScaleTransitions scaleTransitions = new SmallStrain3Dto2DplaneStressScaleTransition(); //TODO: mporoume na to dinoume ston constructor
		Random rnd1 = new Random();
        //private readonly Func<Model, ISolver> createSolver;

        // aparaithta gia to implementation tou IFiniteElementMaterial3D
        double[,] constitutiveMatrix;
		private double[] trueStressVec; // Todo rename the stresses variable
		Matrix transformationMatrix; // gia to shell
		private bool modified; // opws sto MohrCoulomb gia to modified

        private double[,] Cijrs_prev;
        private bool matrices_not_initialized = true;
        private double tol;
        public void InitializeMatrices()
        {
            Cijrs_prev = new double[3,3];
            matrices_not_initialized = false;
            tol = Math.Pow(10, -19);
            constitutiveMatrix = new double[3,3];
			this.CalculateTransformationMatrix(Vector.CreateFromArray(TangentVectorV1), Vector.CreateFromArray(TangentVectorV2));
		}

        private GenericConstitutiveLawState currentState;
        public GenericConstitutiveLawState CurrentState
        {
            get =>  throw new NotSupportedException(); //TODO exei noma mono gia to full linear response RVE
            set=> throw new NotSupportedException(); 
            
        }
        IHaveState ICreateState.CreateState() => CreateState();


        //double[] Stresses { get; }
        //IMatrix2D ConstitutiveMatrix { get; } TODOGerasimos

        //Random properties 
        private int database_size;

		public MicrostructureShell2D(IdegenerateRVEbuilder rveBuilder, /*Func<Model, ISolver> createSolver,*/ 
            bool EstimateOnlyLinearResponse, int database_size, IAlgebraicStrategy<TMatrix> algebraicStrategy)
        {
            this.rveBuilder = rveBuilder;
            //this.createSolver = createSolver;
            this.EstimateOnlyLinearResponse = EstimateOnlyLinearResponse;
            this.database_size = database_size;
            this.algebraicStrategy = algebraicStrategy;
        }

        private void InitializeData()
        {
            Tuple<Model, Dictionary<int, INode>, double> modelAndBoundaryNodes = this.rveBuilder.GetModelAndBoundaryNodes();
            this.model = modelAndBoundaryNodes.Item1;
            this.boundaryNodes = modelAndBoundaryNodes.Item2;
            this.boundaryElements = GetSubdomainsBoundaryFiniteElementsDictionaries(model, boundaryNodes);
            this.volume = modelAndBoundaryNodes.Item3;
            DefineAppropriateConstraintsForBoundaryNodes();
            this.model.ConnectDataStructures();

            // TODO Gerasimos this is a temporar position for the solve creation. It should be done in its original position
            //in the method update material stn arxh pou ginetai to initialization twn diforwn entities 
            (solver, globalAlgebraicModel) = algebraicStrategy.GetSolver(model);
            provider = new ProblemStructural(model, globalAlgebraicModel, solver); //TODO ger mv1: to pou vrisketai h dhmiourgia tou problem structural prin to orderdofs dld//.
            globalAlgebraicModel.OrderDofs();
        }


        private void DefineAppropriateConstraintsForBoundaryNodes()
        {
			var RigidBodyNodeConstraints = rveBuilder.GetModelRigidBodyNodeConstraints(model);
			foreach (Node boundaryNode in boundaryNodes.Values)
			{
				scaleTransitions.ImposeAppropriateAndRigidBodyConstraintsPerBoundaryNode(model, boundaryNode, RigidBodyNodeConstraints);
			}
		}

        private void InitializeFreeAndPrescribedDofsInitialDisplacementVectors()
        {
            uInitialFreeDOFDisplacementsPerSubdomain = globalAlgebraicModel.CreateZeroVector(); //new Dictionary<int, IVector>();
																								//uInitialFreeDOFDisplacementsPerSubdomain = new Dictionary<int, IVector>();
																								//foreach(Subdomain subdomain in model.SubdomainsDictionary.Values)
																								//{
																								//    uInitialFreeDOFDisplacementsPerSubdomain.Add(subdomain.ID, Vector.CreateZero(subdomain.FreeDofOrdering.NumFreeDofs));// prosoxh sto Id twn subdomain
																								//}
			double[] smallStrainVec = new double[3];
			initialConvergedBoundaryDisplacements = new Dictionary<int, Dictionary<IDofType, double>>();
            foreach (Node boundaryNode in boundaryNodes.Values)
            {
                scaleTransitions.ModifyMicrostructureTotalPrescribedBoundaryDisplacementsVectorForMacroStrainVariable(boundaryNode,
                smallStrainVec, initialConvergedBoundaryDisplacements);
            }            
        }

		public IShellMaterial Clone()
		{
			Random rnd1 = new Random();
			int new_rve_id = rnd1.Next(1, database_size + 1);
			return new MicrostructureShell2D<TMatrix>((IdegenerateRVEbuilder)rveBuilder.Clone(new_rve_id), EstimateOnlyLinearResponse, database_size, algebraicStrategy);
		}

		object ICloneable.Clone() => this.Clone();

		public Dictionary<int, INode> BoundaryNodesDictionary
        {
            get { return boundaryNodes; }
        }
        public IList<INode> BoundaryNodes
        {
            get { return boundaryNodes.Values.ToList<INode>(); }
        }

        public double[] UpdateConstitutiveMatrixAndEvaluateResponse(double[] smallStrainVec)
        {
            
            if (matrices_not_initialized)
            {
                this.InitializeMatrices();
                this.InitializeData();


                //TODO Ger: to pou vrisketai h dhmiourgia tou problem structural prin to orderdofs dld
                //TODO Ger: The disposable solver sould be create here or in the else section
                //solver = createSolver(model);
                //solver.OrderDofs(false);
                //foreach (ILinearSystem linearSystem in solver.LinearSystems.Values)
                //{
                //    linearSystem.Reset(); //TODO find out if new structures cause any problems
                //    linearSystem.Subdomain.Forces = Vector.CreateZero(linearSystem.Size);
                //}
                solver.LinearSystem.RhsVector.Clear(); //TODO Ger Temporary fix add watch
                provider.Reset();
                this.InitializeFreeAndPrescribedDofsInitialDisplacementVectors();
            }
            else
            {
				if (!(solver.LinearSystem.RhsVector is null))
				{
					solver.LinearSystem.RhsVector.Clear();
					if (!(PeviousResidual is null))
					{
						solver.LinearSystem.RhsVector = PeviousResidual;
					}
				}
				//solver.LinearSystem.RhsVector.Clear(); //TODO Ger Temporary fix add watch
				provider.Reset();


                //TODO Ger: or the disposable solver sould be create here 
                //solver = createSolver(model);
                //solver.OrderDofs(false); //v2.1. TODO: Is this needed in this case?
                //foreach (ILinearSystem linearSystem in solver.LinearSystems.Values)
                //{
                //    linearSystem.Reset();

                //TODO GER the following capability should also be restored
                // alla vevaia proswrina xrsimopoioume to gegonos oti den petitetai o solver

                //    linearSystem.RhsVector = linearSystem.Subdomain.Forces; //TODO MS 
                //}
            }

			double[] rveCoordinatesSmallStrainVec = TransformStrains(smallStrainVec);
			smallStrainVec = rveCoordinatesSmallStrainVec;


			for (int i1 = 0; i1 < 3; i1++)
            {
                for (int j1 = 0; j1 < 3; j1++)
                {Cijrs_prev[i1, j1] = constitutiveMatrix[i1, j1];}
            }

            #region Rve prescribed Dofs total DIsplacement Dictionary Creation (nessesary for NRNLAnalyzer)
            Dictionary<int, Dictionary<IDofType, double>> totalPrescribedBoundaryDisplacements = new Dictionary<int, Dictionary<IDofType, double>>();
            foreach (Node boundaryNode in boundaryNodes.Values)
            {
                scaleTransitions.ModifyMicrostructureTotalPrescribedBoundaryDisplacementsVectorForMacroStrainVariable(boundaryNode,
                smallStrainVec, totalPrescribedBoundaryDisplacements);
            }
            #endregion
                     
            //var linearSystems = CreateNecessaryLinearSystems(model);    // OPOU pairnei rhs apo subdomainForces       
            //var solver = GetAppropriateSolver(linearSystems);

            
            #region Creation of nessesary analyzers for NRNLAnalyzer and Creation of Microstructure analyzer (NRNLdevelop temporarilly) and solution ;
            int increments = 1; int MaxIterations = 100; int IterationsForMatrixRebuild = 1;
            (DisplacementBvpNRNLAnalyzer microAnalyzer, ElementStructuralStiffnessProvider elementProvider) = 
                AnalyzeMicrostructure(model, solver, increments, MaxIterations, IterationsForMatrixRebuild,
                totalPrescribedBoundaryDisplacements, initialConvergedBoundaryDisplacements, boundaryNodes, uInitialFreeDOFDisplacementsPerSubdomain, globalAlgebraicModel, provider);
            #endregion

            #region update of free converged displacements vectors
            uInitialFreeDOFDisplacementsPerSubdomain = microAnalyzer.GetConvergedSolutionVectorsOfFreeDofs();// ousiastika to u pou twra taftizetai me to uPlusuu
			PeviousResidual = solver.LinearSystem.RhsVector.Copy();
			#endregion


			#region INTEGRATION stresses 
			IGlobalVector du = microAnalyzer.GetConvergedIncrementalSolutionVectorsOfFreeDofs();
            var boundaryNodesIds = boundaryNodes.Keys.ToList();
            Dictionary<int, double[]> FppReactionVectorSubdomains = GenericSubdomainCalculationsMultiple<TMatrix>.CalculateFppReactionsVectorSubdomains(model, elementProvider, scaleTransitions, boundaryNodes, boundaryNodesIds,
                uInitialFreeDOFDisplacementsPerSubdomain, initialConvergedBoundaryDisplacements, totalPrescribedBoundaryDisplacements, increments, increments, globalAlgebraicModel);
            double[] FppReactionVector= SubdomainCalculationsMultiple.CombineMultipleSubdomainsStressesIntegrationVectorsIntoTotal(FppReactionVectorSubdomains);



            double[] DqFpp = SubdomainCalculations.CalculateDqFpp(FppReactionVector, scaleTransitions, boundaryNodes);

			trueStressVec = new double[DqFpp.Length];
			for (int i1 = 0; i1 < DqFpp.Length; i1++)
			{ trueStressVec[i1] = (1 / volume) * DqFpp[i1]; }
			//TODOna elegxthei h parapanw anadiataxh kai o pollaplasiasmos
			#endregion

			#region INTEGRATION constitutive Matrix
			var integrationSimultaneous = new GenericSubdomainCalculationsAndAssembly<TMatrix>();
            (IGlobalVector[] KfpDqSubdomains, double[][] KppDqVectors) = 
                integrationSimultaneous.UpdateSubdomainKffAndCalculateKfpDqAndKppDqpMultipleObje(model, elementProvider, scaleTransitions, boundaryNodes, boundaryElements, solver, globalAlgebraicModel);


            IGlobalVector[] f2_vectorsSubdomains = GenericSubdomainCalculationsMultiple<TMatrix>.CalculateKffinverseKfpDqSubdomains(KfpDqSubdomains, model, elementProvider, scaleTransitions, boundaryNodes, solver);

            double[][] f3_vectors = GenericSubdomainCalculations<TMatrix>.CalculateKpfKffinverseKfpDq(f2_vectorsSubdomains, model.EnumerateSubdomains().First(),
                elementProvider, scaleTransitions, boundaryNodes, globalAlgebraicModel);

            // TODO GEr maintain te following lines for future multi subdomain implemntation
            //double[][] f3_vectors = SubdomainCalculationsMultiple.CombineMultipleSubdomainsIntegrationVectorsIntoTotal(f3_vectorsSubdomains,scaleTransitions);
            //double[][] KppDqVectors = SubdomainCalculationsMultiple.CombineMultipleSubdomainsIntegrationVectorsIntoTotal(KppDqVectorsSubdomains,scaleTransitions);

            double[][] f4_vectors = SubdomainCalculations.SubtractConsecutiveVectors(KppDqVectors, f3_vectors);
            double[,] DqCondDq = SubdomainCalculations.CalculateDqCondDq(f4_vectors, scaleTransitions, boundaryNodes);

			double[,] constitutiveMat = new double[DqCondDq.GetLength(0), DqCondDq.GetLength(1)];
			for (int i1 = 0; i1 < DqCondDq.GetLength(0); i1++)
			{
				for (int i2 = 0; i2 < DqCondDq.GetLength(1); i2++)
				{
					constitutiveMat[i1, i2] = (1 / volume) * DqCondDq[i1, i2];
				}
			}
			#endregion

			#region update of prescribed converged displacements vectors;
			initialConvergedBoundaryDisplacements = totalPrescribedBoundaryDisplacements;
			#endregion

			#region constitutive tensors transformation methods
			// transformation gia to shell 
			(var transformedTrueStressVec, var transformedConstitutiveMat) = StressesAndConstitutiveMatrixTransformation(trueStressVec, constitutiveMat);
			#endregion

			this.constitutiveMatrix = transformedConstitutiveMat;
			trueStressVec = transformedTrueStressVec;

			//PrintMethodsForDebug(KfpDq, f2_vectors, f3_vectors, KppDqVectors, f4_vectors, DqCondDq, d2W_dfdf, Cijrs);
			this.modified = CheckIfConstitutiveMatrixChanged();

            return trueStressVec;
        }

		private void CalculateTransformationMatrix(Vector surfaceBasisVector1, Vector surfaceBasisVector2)
		{
			var auxMatrix1 = Matrix.CreateZero(2, 2);  //auxMatrix: covariant metric coefficients gab
			auxMatrix1[0, 0] = surfaceBasisVector1.DotProduct(surfaceBasisVector1);
			auxMatrix1[0, 1] = surfaceBasisVector1.DotProduct(surfaceBasisVector2);
			auxMatrix1[1, 0] = surfaceBasisVector2.DotProduct(surfaceBasisVector1);
			auxMatrix1[1, 1] = surfaceBasisVector2.DotProduct(surfaceBasisVector2);
			Matrix inverse = auxMatrix1.Invert(); //inverse: contravariant metric coefficients g_ab (ekthetis ta a,b)

			//Contravariant base vectors
			double[][] G_i = new double[2][];
			for (int i1 = 0; i1 < 2; i1++)
			{
				G_i[i1] = new double[3];
				for (int i2 = 0; i2 < 3; i2++)
				{
					G_i[i1][i2] = inverse[i1, 0] * surfaceBasisVector1[i2] + inverse[i1, 1] * surfaceBasisVector2[i2];
				}
			}

			//Normalised covariant base vectors
			double[][] Ei = new double[2][];// to trito den xreiazetai

			Ei[0] = surfaceBasisVector1.CopyToArray();
			double G1_norm = surfaceBasisVector1.Norm2();
			for (int i1 = 0; i1 < 3; i1++) { Ei[0][i1] = Ei[0][i1] / G1_norm; }

			double G2_dot_E1 = 0;
			for (int i1 = 0; i1 < 3; i1++) { G2_dot_E1 += surfaceBasisVector2[i1] * Ei[0][i1]; }

			double[] projection = new double[3];
			for (int i1 = 0; i1 < 3; i1++) { projection[i1] = G2_dot_E1 * Ei[0][i1]; }

			Ei[1] = new double[3];
			for (int i1 = 0; i1 < 3; i1++) { Ei[1][i1] = surfaceBasisVector2[i1] - projection[i1]; }
			double norm1 = (Vector.CreateFromArray(Ei[1])).Norm2();
			for (int i1 = 0; i1 < 3; i1++) { Ei[1][i1] = Ei[1][i1] / norm1; }

			double[,] EiDOTG_j = new double[2, 2];

			for (int i1 = 0; i1 < 2; i1++)
			{
				for (int i2 = 0; i2 < 2; i2++)
				{
					EiDOTG_j[i1, i2] = Vector.CreateFromArray(Ei[i1]).DotProduct(Vector.CreateFromArray(G_i[i2]));
				}
			}

			transformationMatrix = Matrix.CreateFromArray(new double[3, 3] { {EiDOTG_j[0,0]*EiDOTG_j[0,0],EiDOTG_j[0,1]*EiDOTG_j[0,1],EiDOTG_j[0,0]*EiDOTG_j[0,1]  },
				 {EiDOTG_j[1,0]*EiDOTG_j[1,0],EiDOTG_j[1,1]*EiDOTG_j[1,1],EiDOTG_j[1,0]*EiDOTG_j[1,1]  },
				{2*EiDOTG_j[1,0]*EiDOTG_j[0,0],2*EiDOTG_j[1,1]*EiDOTG_j[0,1],EiDOTG_j[1,0]*EiDOTG_j[0,1]+EiDOTG_j[1,1]*EiDOTG_j[0,0]   } });
		}

		private double[] TransformStrains(double[] smallStrainVec)
		{
			double[] rveCoordinatesSmallStrainVec = (transformationMatrix * (Vector.CreateFromArray(smallStrainVec))).CopyToArray();
			return rveCoordinatesSmallStrainVec;
		}


		private (double[], double[,]) StressesAndConstitutiveMatrixTransformation(double[] trueStressVec, double[,] constitutiveMat)
		{
			var transformedTrueStressVec = (transformationMatrix.Transpose() * (Vector.CreateFromArray(trueStressVec))).CopyToArray();
			// TODO: CHECK matrix for corrupted data after multiplication
			var transformedConstitutiveMat = (transformationMatrix.Transpose() * (Matrix.CreateFromArray(constitutiveMat)) * transformationMatrix).CopyToArray2D();

			return (transformedTrueStressVec, transformedConstitutiveMat);
		}

		private bool CheckIfConstitutiveMatrixChanged()
        {
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                    if (Math.Abs(Cijrs_prev[i, j] - constitutiveMatrix[i, j]) > 1e-10)
                        return true;

            return false;
        }



        #region IFiniteElementMaterial3D methodoi mia mia 

        public IMatrixView ConstitutiveMatrix
        {
            get
            {
                if (constitutiveMatrix == null) CalculateOriginalConstitutiveMatrixWithoutNLAnalysis(); // TODOGerasimos arxiko constitutive mporei na upologizetai pio efkola
                return Matrix.CreateFromArray(constitutiveMatrix); // TODO: apla kratame to constitutive matrix san array[,] (alla matrix mporei na xrhimopoithei gia tis peristrofes)
			}
        }

        public double[] Stresses // opws xrhsimopoeitai sto mohrcoulomb kai hexa8
        {
            get { return trueStressVec; }
        }

        public GenericConstitutiveLawState CreateState()//.1`
        {
            var subdomainUpdaters = new NonLinearModelUpdaterWithInitialConditions(globalAlgebraicModel);
            subdomainUpdaters.UpdateState();
            //var subdomainUpdaters = new Dictionary<int, NonLinearSubdomainUpdaterWithInitialConditions>(1); 
            //foreach (Subdomain subdomain in model.SubdomainsDictionary.Values){subdomainUpdaters.Add(subdomain.ID, new NonLinearSubdomainUpdaterWithInitialConditions(subdomain)); //v2.3}
            //foreach (var subdomainUpdater in subdomainUpdaters.Values){subdomainUpdater.UpdateState();}

            currentState = new GenericConstitutiveLawState(this, new (string, double)[0]); // TODO: an xreiazottan pote tetoia ulopoihsh tote tha itan to tupou  {(HARDENING_X, alpha[0]),(HARDENING_Y, alpha[1])}
            //klp san to bondslip coesive materials kai ta eprepe na perilamvanei tou rve oles tis state variables dld kai ta materials kai tis free metakinhseis twn elements kai pitano contact klp kai ta uprescribed
                

            return currentState;


        }

        public bool Modified
        {
            get { return modified; }
        }

        public void ResetModified()
        {
            modified = false;
        }

        public int ID
        {
            get { return 1000; }
        }


        #endregion
        // methodoi ews edw xrhsimopoiountai
        public void ClearState() 
        {
            // pithanws TODO 
        }
        public void ClearStresses()
        {
            // pithanws TODO 
        }
        public double[] Coordinates { get; set; }

        public double YoungModulus => throw new NotSupportedException();

        public double PoissonRatio => throw new NotSupportedException();


        

        public void CalculateOriginalConstitutiveMatrixWithoutNLAnalysis()
        {
            //ISolver solver;
            if (matrices_not_initialized)
            {
                this.InitializeMatrices();
                this.InitializeData();
				//TODO Ger: to pou vrisketai h dhmiourgia tou problem structural prin to orderdofs dld
				//TODO Ger: The disposable solver sould be create here or in the else section
				//solver = createSolver(model);
				//solver.OrderDofs(false); //model.GlobalDofOrdering = solver.DofOrderer.OrderDofs(model); //TODO find out if new structures cause any problems
				//foreach (ILinearSystem linearSystem in solver.LinearSystems.Values)
				//{
				//    linearSystem.Reset(); //TODO find out if new structures cause any problems
				//    linearSystem.Subdomain.Forces = Vector.CreateZero(linearSystem.Size);
				//}
				this.InitializeFreeAndPrescribedDofsInitialDisplacementVectors();
            }
            else
            {
                //solver = createSolver(model);
                //solver.OrderDofs(false); //v2.1. TODO: Is this needed in this case?
                //foreach (ILinearSystem linearSystem in solver.LinearSystems.Values) linearSystem.Reset();
                ////solver.ResetSubdomainForcesVector();
            }


			trueStressVec = new double[3];
			var elementProvider = new ElementStructuralStiffnessProvider();
			#region INTEGRATION constitutive Matrix            
			var integrationSimultaneous = new GenericSubdomainCalculationsAndAssembly<TMatrix>();
            (IGlobalVector[] KfpDqSubdomains, double[][] KppDqVectors) =
                integrationSimultaneous.UpdateSubdomainKffAndCalculateKfpDqAndKppDqpMultipleObje(model, elementProvider, scaleTransitions, boundaryNodes, boundaryElements, solver, globalAlgebraicModel);

            IGlobalVector[] f2_vectorsSubdomains = GenericSubdomainCalculationsMultiple<TMatrix>.CalculateKffinverseKfpDqSubdomains(KfpDqSubdomains, model, elementProvider, scaleTransitions, boundaryNodes, solver);

            double[][] f3_vectors = GenericSubdomainCalculations<TMatrix>.CalculateKpfKffinverseKfpDq(f2_vectorsSubdomains, model.EnumerateSubdomains().First(),
                elementProvider, scaleTransitions, boundaryNodes, globalAlgebraicModel);

            // TODO GEr maintain te following lines for future multi subdomain implemntation
            //double[][] f3_vectors = SubdomainCalculationsMultiple.CombineMultipleSubdomainsIntegrationVectorsIntoTotal(f3_vectorsSubdomains, scaleTransitions);
            //double[][] KppDqVectors = SubdomainCalculationsMultiple.CombineMultipleSubdomainsIntegrationVectorsIntoTotal(KppDqVectorsSubdomains, scaleTransitions);

            double[][] f4_vectors = SubdomainCalculations.SubtractConsecutiveVectors(KppDqVectors, f3_vectors);
            double[,] DqCondDq = SubdomainCalculations.CalculateDqCondDq(f4_vectors, scaleTransitions, boundaryNodes);

			double[,] constitutiveMat = new double[DqCondDq.GetLength(0), DqCondDq.GetLength(1)];
			for (int i1 = 0; i1 < DqCondDq.GetLength(0); i1++)
			{
				for (int i2 = 0; i2 < DqCondDq.GetLength(1); i2++)
				{
					constitutiveMat[i1, i2] = (1 / volume) * DqCondDq[i1, i2];
				}
			}

			#endregion

			#region constitutive tensors transformation methods
			// transformation gia to shell 
			(var transformedTrueStressVec, var transformedConstitutiveMat) = StressesAndConstitutiveMatrixTransformation(trueStressVec, constitutiveMat);
			this.constitutiveMatrix = transformedConstitutiveMat;
			trueStressVec = transformedTrueStressVec;
			#endregion

			//PrintMethodsForDebug(KfpDq, f2_vectors, f3_vectors, KppDqVectors, f4_vectors, DqCondDq, d2W_dfdf, Cijrs);
			this.modified = CheckIfConstitutiveMatrixChanged();

            if (EstimateOnlyLinearResponse)
            {
				model = null;
				boundaryElements = null;
				boundaryNodes = null;
				NormalVectorV3 = null;
				TangentVectorV1 = null;
				TangentVectorV2 = null;
				rveBuilder = null;
				uInitialFreeDOFDisplacementsPerSubdomain = null;
				initialConvergedBoundaryDisplacements = null;
				trueStressVec = null;
				transformationMatrix = null;
				Cijrs_prev = null;
			}
        }

		#region transformation methods
		//TODO: implement and use methods for shell transformation but they can be placed externally in a class for all 2D materials to be used from shells.
		//  or delete if unenecessary
		private double[] transformTrueStressVec(double[] trueStressVec, double[] tangent1, double[] tangent2, double[] normal)
		{
			throw new NotImplementedException();
		}

		private double[,] TransformConstitutiveMatrix(double[,] constitutiveMat, double[] tangent1, double[] tangent2, double[] normal)
		{
			throw new NotImplementedException();
		}

		#endregion

		#region Print methods for debug
		public double[] RetrieveDisplacementsOfFreeDofs()
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

        public NodalResults RetrieveAndCorrectNodalData()
        {
            throw new NotImplementedException();
            //TODO implement tihs
        }
        //private void PrintMethodsForDebug(double[][] KfpDq, double[][] f2_vectors, double[][] f3_vectors, double[][] KppDqVectors, double[][] f4_vectors, double[,] DqCondDq, double[,] d2W_dfdf, double[,] Cijrs)
        //{
        //    string string0 = @"C:\Users\turbo-x\Desktop\notes_elegxoi\REFERENCE_kanonikh_gewmetria_2\REF2_10__000_renu_new_multiple_algorithms_check_develop_1GrSh_correct_coh_CHECK_integration\d2\";

        //    string string1 = String.Concat(string0, @"KfpDq_{0}.txt");

        //    for (int i2 = 0; i2 < KfpDq.GetLength(0); i2++)
        //    {
        //        string path = string.Format(string1, (i2 + 1).ToString());
        //        Vector data = new Vector(KfpDq[i2]);
        //        data.WriteToFile(path);
        //    }

        //    string string2 = String.Concat(string0, @"KffInvKfpDq_{0}.txt");



        //    for (int i2 = 0; i2 < f2_vectors.GetLength(0); i2++)
        //    {
        //        string path = string.Format(string2, (i2 + 1).ToString());
        //        Vector data = new Vector(f2_vectors[i2]);
        //        data.WriteToFile(path);
        //    }

        //    string string3 = String.Concat(string0, @"f3_vectors_{0}.txt");
        //    string string4 = String.Concat(string0, @"KppDqVectors_{0}.txt");
        //    string string5 = String.Concat(string0, @"f4_vectors_{0}.txt");

        //    for (int i2 = 0; i2 < f2_vectors.GetLength(0); i2++)
        //    {
        //        string path = string.Format(string3, (i2 + 1).ToString());
        //        Vector data = new Vector(f3_vectors[i2]);
        //        data.WriteToFile(path);

        //        path = string.Format(string4, (i2 + 1).ToString());
        //        data = new Vector(KppDqVectors[i2]);
        //        data.WriteToFile(path);

        //        path = string.Format(string5, (i2 + 1).ToString());
        //        data = new Vector(f4_vectors[i2]);
        //        data.WriteToFile(path);

        //    }

        //    PrintUtilities.WriteToFile(DqCondDq, String.Concat(string0, @"DqCondDq.txt"));
        //    PrintUtilities.WriteToFile(d2W_dfdf,  String.Concat(string0, @"d2W_dfdf.txt"));
        //    PrintUtilities.WriteToFile(Cijrs, String.Concat(string0, @"Cijrs.txt"));
        //}
        #endregion


    }
	//Microstructure3DevelopMultipleSubdomainsUseBaseSmallStrainsShelltransformationSimuRandObj
	// Origin: Microstructure3DevelopMultipleSubdomainsUseBaseSmallStrainsShelltransformationSimuRand
	// modifications: --> updated se v2



}
