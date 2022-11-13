using System;
using System.Collections.Generic;
using System.Diagnostics;
using ISAAR.MSolve.FEM.Elements;
using MGroup.Constitutive.Structural;
using MGroup.Constitutive.Structural.BoundaryConditions;
using MGroup.Constitutive.Structural.Cohesive;
using MGroup.Constitutive.Structural.Continuum;
using MGroup.Constitutive.Structural.Transient;
using MGroup.FEM.Structural.Continuum;
using MGroup.MSolve.Discretization;
using MGroup.MSolve.Discretization.Dofs;
using MGroup.MSolve.Discretization.Entities;
using MGroup.MSolve.MultiscaleAnalysis.Interfaces;
using MGroup.MSolve.Numerics.Integration.Quadratures;
using MGroup.MSolve.Numerics.Interpolation;
using MGroup.Multiscale.SupportiveClasses;

namespace ISAAR.MSolve.MultiscaleAnalysis
{

    public class GmshCompositeRveBuilderNL_DefGradCohesive : IdegenerateRVEbuilder
    {// test 3d
        double L01, L02, L03 ;
        double boundarySearchTol;
        //IIsotropicContinuumMaterial3D matrixMaterial;
        //IIsotropicContinuumMaterial3D inclusionMaterial;
        IContinuumMaterial3DDefGrad matrixMaterial;
        IContinuumMaterial3DDefGrad inclusionMaterial;


        string gmshRveFilePath;

        double[,] rveNodeData;
        List<int[,]> celltype2ELementsAndNodes;
        List<int[,]> celltype4ELementsAndNodes;
        int[] boundaryNodesIds1;
        List<int>   rigidBodyConstraintNodeIds;
        ICohesiveZoneMaterial cohesiveMaterial;

        //TODO: input material to be cloned.  

        int[,] TetInterfaceCohesiveNodeIds;

        double[,] duplicateNodesIdsAndCoordinates;


        public GmshCompositeRveBuilderNL_DefGradCohesive(/*IIsotropicContinuumMaterial3D matrixMaterial, IIsotropicContinuumMaterial3D inclusionMaterial,*/
            IContinuumMaterial3DDefGrad matrixMaterial, IContinuumMaterial3DDefGrad inclusionMaterial,
            double L01, double L02, double L03, ICohesiveZoneMaterial cohesiveMaterial, string gmshRveFilePath, double boundarySearchTol = 1e-07)
        {
            this.matrixMaterial = matrixMaterial;
            this.inclusionMaterial = inclusionMaterial;
            this.L01 = L01;
            this.L02 = L02;
            this.L03 = L03;
            this.boundarySearchTol = boundarySearchTol;
            this.gmshRveFilePath = gmshRveFilePath;

            (this.rveNodeData, this.celltype2ELementsAndNodes, this.celltype4ELementsAndNodes) = GmshMultiplePhaseReader.ReadFile(gmshRveFilePath, false);

            // This will correct  (overwrite) duplicate node info in celltype4ELementsAndNodes as well.
            (this.TetInterfaceCohesiveNodeIds, this.duplicateNodesIdsAndCoordinates) = GmshMultiplePhaseReader.SearchCohesiveInterfacesElements(rveNodeData,
            celltype2ELementsAndNodes, celltype4ELementsAndNodes);

            ( boundaryNodesIds1, rigidBodyConstraintNodeIds) = SearchBoundaryNodesIds1(rveNodeData, L01, boundarySearchTol);

            this.cohesiveMaterial = cohesiveMaterial;
        }

        

        public GmshCompositeRveBuilderNL_DefGradCohesive(/*IIsotropicContinuumMaterial3D outterMaterial, IIsotropicContinuumMaterial3D inclusionMaterial,*/
            IContinuumMaterial3DDefGrad outterMaterial, IContinuumMaterial3DDefGrad innerMaterial,
            double L01, double L02, double L03, string gmshRveFilePath, double[,] rveNodeData, List<int[,]> celltype2ELementsAndNodes, List<int[,]> celltype4ELementsAndNodes, int[] boundaryNodesIds1, ICohesiveZoneMaterial cohesiveMaterial, List<int> rigidBodyConstraintNodeIds,
             int[,] TetInterfaceCohesiveNodeIds, double[,] duplicateNodesIdsAndCoordinates, double boundarySearchTol = 1e-09)
        {
            this.matrixMaterial = outterMaterial;
            this.inclusionMaterial = innerMaterial;
            this.L01 = L01;
            this.L02 = L02;
            this.L03 = L03;
            this.boundarySearchTol = boundarySearchTol;
            this.gmshRveFilePath = gmshRveFilePath;

            this.rveNodeData = rveNodeData;
            this.celltype2ELementsAndNodes = celltype2ELementsAndNodes;
            this.celltype4ELementsAndNodes = celltype4ELementsAndNodes;
            this.boundaryNodesIds1 = boundaryNodesIds1;
            this.rigidBodyConstraintNodeIds = rigidBodyConstraintNodeIds;
            //.

            //.
            this.TetInterfaceCohesiveNodeIds = TetInterfaceCohesiveNodeIds;
            this.duplicateNodesIdsAndCoordinates = duplicateNodesIdsAndCoordinates;

            this.cohesiveMaterial = cohesiveMaterial;

        }

        public IRVEbuilder Clone(int a)
        {
            return new GmshCompositeRveBuilderNL_DefGradCohesive(matrixMaterial,inclusionMaterial,
                L01, L02, L03, gmshRveFilePath,  rveNodeData, celltype2ELementsAndNodes, celltype4ELementsAndNodes, boundaryNodesIds1, cohesiveMaterial, rigidBodyConstraintNodeIds, TetInterfaceCohesiveNodeIds, duplicateNodesIdsAndCoordinates, boundarySearchTol);
        }

        Tuple<Model, Dictionary<int, INode>, double> IRVEbuilder.GetModelAndBoundaryNodes()
        {
            Model model = new Model();
            
            model.SubdomainsDictionary[0] = new Subdomain(0);
       
            //var (Tet_Outter_elements_Node_data, Tet_Inner_elements_Node_data, node_coords, NodeIds, boundaryNodesIds, rigidNodes) =
            //    GetModelCreationData();

            for (int i1 = 0; i1 < rveNodeData.GetLength(0); i1++)
            {
                int nodeID = (int)rveNodeData[i1,0];
                double nodeCoordX = rveNodeData[i1, 1];
                double nodeCoordY = rveNodeData[i1, 2];
                double nodeCoordZ = rveNodeData[i1, 3];

                //if (!CnstValues.useV2FiniteElements)
                //{
                model.NodesDictionary.Add(nodeID, new Node(id: nodeID, x: nodeCoordX, y: nodeCoordY, z: nodeCoordZ));
                //}
                //else
                //{
                //    model.NodesDictionary.Add(nodeID, new Node(id: nodeID, x: nodeCoordX, y: nodeCoordY, z: nodeCoordZ)
                //    {
                //        oX = new double[] { nodeCoordX, nodeCoordY, nodeCoordZ },
                //        tX = new double[] { nodeCoordX, nodeCoordY, nodeCoordZ },
                //        tU = new double[3]
                //    });
                //}
            }

            // region extra nodes for interface
            for (int i1 = 0; i1 < duplicateNodesIdsAndCoordinates.GetLength(0); i1++)
            {
                int nodeID = (int)duplicateNodesIdsAndCoordinates[i1, 0];
                double nodeCoordX = duplicateNodesIdsAndCoordinates[i1, 1];
                double nodeCoordY = duplicateNodesIdsAndCoordinates[i1, 2];
                double nodeCoordZ = duplicateNodesIdsAndCoordinates[i1, 3];

                //if (!CnstValues.useV2FiniteElements)
                //{
                model.NodesDictionary.Add(nodeID, new Node(id: nodeID, x: nodeCoordX, y: nodeCoordY, z: nodeCoordZ));
                //}
                //else
                //{
                //    model.NodesDictionary.Add(nodeID, new Node(id: nodeID, x: nodeCoordX, y: nodeCoordY, z: nodeCoordZ)
                //    {
                //        oX = new double[] { nodeCoordX, nodeCoordY, nodeCoordZ },
                //        tX = new double[] { nodeCoordX, nodeCoordY, nodeCoordZ },
                //        tU = new double[3]
                //    });
                //}
            }

            int nElementGroupsI = celltype4ELementsAndNodes.Count;

            //define outer elements group//.
            int[] ContinuumTet4NodesNumbering = new int[4] { 0, 1, 2, 3 };
            int subdomainID = 0;
			//var factoryOutter = new ContinuumElement3DFactory(matrixMaterial, null);
			TransientAnalysisProperties dynamicMaterial = new TransientAnalysisProperties(1, 0, 0);
            for (int i1 = 0; i1 < celltype4ELementsAndNodes[nElementGroupsI-1].GetLength(0); i1++)
            {
                List<Node> nodeSet = new List<Node>();
                for (int j = 0; j < 4; j++)
                {
                    int ren1 = ContinuumTet4NodesNumbering[j];
                    int nodeID = celltype4ELementsAndNodes[nElementGroupsI-1][i1, ren1+1];
                    nodeSet.Add((Node)model.NodesDictionary[nodeID]);
                }

				IElementType e2 = new ContinuumElement3DNonLinearDefGrad(nodeSet, matrixMaterial, TetrahedronQuadrature.Order1Point1, InterpolationTet4.UniqueInstance);//.//.
				e2.ID = celltype4ELementsAndNodes[nElementGroupsI - 1][i1, 0];
				//{
                    //ID = celltype4ELementsAndNodes[nElementGroupsI - 1][i1, 0],
                    //ElementType = factoryOutter.CreateElement(CellType.Tet4, nodeSet) // dixws to e. exoume sfalma enw sto beambuilding oxi//edw kaleitai me ena orisma to Hexa8
                    //ElementType = factoryOutter.CreateNonLinearElement(CellType.Tet4, nodeSet, matrixMaterial, DynamicMaterial)
                    //ElementType = new ContinuumElement3DNonLinearDefGrad(nodeSet, matrixMaterial, TetrahedronQuadrature.Order1Point1, InterpolationTet4.UniqueInstance),//.//.
                    //WARNING cannot use  tetrahedra with GaussLegendre 3D it is necessary to use TetrahedronQuadrature. 
                //};

                //for (int j = 0; j < 4; j++)
                //{
                //    int ren1 = ContinuumTet4NodesNumbering[j];
                //    int nodeID = celltype4ELementsAndNodes[nElementGroupsI-1][i1, ren1+1];
                //    e2.NodesDictionary.Add(nodeID, model.NodesDictionary[nodeID]);
                //}
                model.ElementsDictionary.Add(e2.ID, e2);
                model.SubdomainsDictionary[subdomainID].Elements.Add(e2);
            }

            //define inner elements group
            //var factoryInner = new ContinuumElement3DFactory(inclusionMaterial, null);
            for (int i2 = 0; i2 < nElementGroupsI - 1; i2++)
            {


                for (int i1 = 0; i1 < celltype4ELementsAndNodes[i2].GetLength(0); i1++)
                {
                    List<Node> nodeSet = new List<Node>();
                    for (int j = 0; j < 4; j++)
                    {
                        int ren1 = ContinuumTet4NodesNumbering[j];
                        int nodeID = celltype4ELementsAndNodes[i2][i1, ren1 + 1];
                        nodeSet.Add((Node)model.NodesDictionary[nodeID]);
                    }

					IElementType e2 = new ContinuumElement3DNonLinearDefGrad(nodeSet, inclusionMaterial, TetrahedronQuadrature.Order1Point1, InterpolationTet4.UniqueInstance);
					e2.ID = celltype4ELementsAndNodes[i2][i1, 0];
					//{
                        //ID = celltype4ELementsAndNodes[i2][i1, 0],
                        //ElementType = factoryInner.CreateElement(CellType.Tet4, nodeSet) // dixws to e. exoume sfalma enw sto beambuilding oxi//edw kaleitai me ena orisma to Hexa8
                        //ElementType = factoryInner.CreateNonLinearElement(CellType.Tet4, nodeSet, matrixMaterial, DynamicMaterial)
                        //ElementType = new ContinuumElement3DNonLinearDefGrad(nodeSet, inclusionMaterial, TetrahedronQuadrature.Order1Point1, InterpolationTet4.UniqueInstance),
                        //WARNING cannot use  tetrahedra with GaussLegendre 3D it is necessary to use TetrahedronQuadrature. 
                    //};

                    //for (int j = 0; j < 4; j++)
                    //{
                    //    int ren1 = ContinuumTet4NodesNumbering[j];
                    //    int nodeID = celltype4ELementsAndNodes[i2][i1, ren1 + 1];
                    //    e2.NodesDictionary.Add(nodeID, model.NodesDictionary[nodeID]);
                    //}
                    model.ElementsDictionary.Add(e2.ID, e2);
                    model.SubdomainsDictionary[subdomainID].Elements.Add(e2);
                }

            }

			//define cohesive elments for interface 1 
			IElementType e1;
            for (int i1 = 0; i1 < TetInterfaceCohesiveNodeIds.GetLength(0); i1++)
            {
				List<INode> nodeSet = new List<INode>();
				for (int j = 0; j < 6; j++)
				{
					nodeSet.Add(model.NodesDictionary[TetInterfaceCohesiveNodeIds[i1, 1 + j]]);
				}

				e1 = new cohesiveElement(nodeSet, cohesiveMaterial, /*GaussLegendre2D.GetQuadratureWithOrder(3, 3)*/ TriangleQuadratureSymmetricGaussian.Order1Point1, InterpolationTri3.UniqueInstance); // dixws to e. exoume sfalma enw sto beambuilding oxi//edw kaleitai me ena orisma to Hexa8
				//{
                    //ID = TetInterfaceCohesiveNodeIds[i1, 0],
                    //ElementType = new cohesiveElement(cohesiveMaterial, /*GaussLegendre2D.GetQuadratureWithOrder(3, 3)*/ TriangleQuadratureSymmetricGaussian.Order1Point1, InterpolationTri3.UniqueInstance) // dixws to e. exoume sfalma enw sto beambuilding oxi//edw kaleitai me ena orisma to Hexa8
                //};
				e1.ID = TetInterfaceCohesiveNodeIds[i1, 0];
				//for (int j = 0; j < 6; j++)
				//{
				//    e1.NodesDictionary.Add(TetInterfaceCohesiveNodeIds[i1, 1 + j], model.NodesDictionary[TetInterfaceCohesiveNodeIds[i1, 1 + j]]);
				//}
				model.ElementsDictionary.Add(e1.ID, e1);
                model.SubdomainsDictionary[subdomainID].Elements.Add(e1);//.
            }

            #region boundary nodes assignment data
            Dictionary<int, INode> boundaryNodes = new Dictionary<int, INode>();
            for (int i = 0; i < boundaryNodesIds1.GetLength(0); i++)
            {
                boundaryNodes.Add(boundaryNodesIds1[i], model.NodesDictionary[boundaryNodesIds1[i]]);
            }

            #endregion
            return new Tuple<Model, Dictionary<int, INode>, double>(model, boundaryNodes, L01*L02*L03); 
        }

		Dictionary<Node, IList<IStructuralDofType>> IdegenerateRVEbuilder.GetModelRigidBodyNodeConstraints(Model model)
        {
			var rigidNodes = RigidNodes;
			Dictionary<Node, IList<IStructuralDofType>> rigidBodyNodeConstraints = new Dictionary<Node, IList<IStructuralDofType>>();
			rigidBodyNodeConstraints.Add((Node)model.NodesDictionary[rigidNodes[0]], new List<IStructuralDofType> { StructuralDof.TranslationX, StructuralDof.TranslationY, StructuralDof.TranslationZ });
			rigidBodyNodeConstraints.Add((Node)model.NodesDictionary[rigidNodes[1]], new List<IStructuralDofType> { StructuralDof.TranslationX, StructuralDof.TranslationY, StructuralDof.TranslationZ });
			rigidBodyNodeConstraints.Add((Node)model.NodesDictionary[rigidNodes[2]], new List<IStructuralDofType> { StructuralDof.TranslationX, StructuralDof.TranslationY, StructuralDof.TranslationZ });

			//Dictionary<Node, IList<IDofType>> RigidBodyNodeConstraints = new Dictionary<Node, IList<IDofType>>();

			//         RigidBodyNodeConstraints.Add(model.NodesDictionary[rigidNodes[0]], new List<IDofType>() { StructuralDof.TranslationX, StructuralDof.TranslationY, StructuralDof.TranslationZ });
			//         RigidBodyNodeConstraints.Add(model.NodesDictionary[rigidNodes[1]], new List<IDofType>() { StructuralDof.TranslationX, StructuralDof.TranslationY, StructuralDof.TranslationZ });
			//         RigidBodyNodeConstraints.Add(model.NodesDictionary[rigidNodes[2]], new List<IDofType>() { StructuralDof.TranslationX, StructuralDof.TranslationY, StructuralDof.TranslationZ });

			return rigidBodyNodeConstraints;
        }



        private List<int> RigidNodes => rigidBodyConstraintNodeIds;

        private (int[] bondaryNodesIds1, List<int> rigidBodyConstraintNodeIds) SearchBoundaryNodesIds1(double[,] rveNodeData, double L01, double boundarySearchTol)
        {
            List<int> boundaryNodesIds = new List<int>();
            List<int> rigidBodyConstraintNodeIds = new List<int>();

            for (int i1 = 0; i1 < rveNodeData.GetLength(0); i1++)
            {
                int nodeId = (int)rveNodeData[i1, 0];
                double X = rveNodeData[i1, 1];
                double Y = rveNodeData[i1, 2];
                double Z = rveNodeData[i1, 3];

                bool isBoundaryNode1 = false;

                if (X<-0.5*L01+boundarySearchTol)
                {                    
                    isBoundaryNode1 = true;
                }

                if (X>+0.5*L01-boundarySearchTol)
                {
                    isBoundaryNode1 = true;
                }

                if (Y < -0.5 * L01 + boundarySearchTol)
                {
                    isBoundaryNode1 = true;
                }

                if (Y > +0.5 * L01 - boundarySearchTol)
                {
                    isBoundaryNode1 = true;
                }

                if (isBoundaryNode1)
                { boundaryNodesIds.Add(nodeId); }

                if (Z < -0.5 * L01 + boundarySearchTol)
                {
                    if((X < -0.5 * L01 + boundarySearchTol)&& (Y < -0.5 * L01 + boundarySearchTol))
                    {
                        rigidBodyConstraintNodeIds.Add(nodeId);
                    }

                    if ((X < -0.5 * L01 + boundarySearchTol) && (Y > +0.5 * L01 - boundarySearchTol))
                    {
                        rigidBodyConstraintNodeIds.Add(nodeId);
                    }

                    if ((X > +0.5 * L01 - boundarySearchTol) && (Y < -0.5 * L01 + boundarySearchTol))
                    {
                        rigidBodyConstraintNodeIds.Add(nodeId);
                    }
                }

                

            }



            return (boundaryNodesIds.ToArray(), rigidBodyConstraintNodeIds);
        }

	}
}
