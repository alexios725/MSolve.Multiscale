using System;
using System.Collections.Generic;
using System.Diagnostics;

using MGroup.Constitutive.Structural;
using MGroup.Constitutive.Structural.Continuum;
using MGroup.FEM.Structural.Continuum;
using MGroup.MSolve.Discretization;
using MGroup.MSolve.Discretization.Dofs;
using MGroup.MSolve.Discretization.Entities;
using MGroup.MSolve.MultiscaleAnalysis.Interfaces;
using MGroup.Multiscale.SupportiveClasses;

using ContinuumElement3DFactory = MGroup.FEM.Structural.Continuum.ContinuumElement3DFactory;

namespace MGroup.Multiscale.RveTemplatesPaper
{

    public class GmshCompositeRveBuilder : IdegenerateRVEbuilder
    {// test 3d
        double L01, L02, L03 ;
        double boundarySearchTol;
        IIsotropicContinuumMaterial3D matrixMaterial;
        IIsotropicContinuumMaterial3D inclusionMaterial;

        string gmshRveFilePath;

        double[,] rveNodeData;
        List<int[,]> celltype2ELementsAndNodes;
        List<int[,]> celltype4ELementsAndNodes;
        int[] boundaryNodesIds1;
        List<int>   rigidBodyConstraintNodeIds;

        //TODO: input material to be cloned.  
        public GmshCompositeRveBuilder(IIsotropicContinuumMaterial3D matrixMaterial, IIsotropicContinuumMaterial3D inclusionMaterial,
            double L01, double L02, double L03, string gmshRveFilePath, double boundarySearchTol = 1e-07)
        {
            this.matrixMaterial = matrixMaterial;
            this.inclusionMaterial = inclusionMaterial;
            this.L01 = L01;
            this.L02 = L02;
            this.L03 = L03;
            this.boundarySearchTol = boundarySearchTol;
            this.gmshRveFilePath = gmshRveFilePath;

            (this.rveNodeData, this.celltype2ELementsAndNodes, this.celltype4ELementsAndNodes) = GmshMultiplePhaseReader.ReadFile(gmshRveFilePath, false);

            ( boundaryNodesIds1, rigidBodyConstraintNodeIds) = SearchBoundaryNodesIds1(rveNodeData, L01, boundarySearchTol);


        }

        

        public GmshCompositeRveBuilder(IIsotropicContinuumMaterial3D outterMaterial, IIsotropicContinuumMaterial3D inclusionMaterial,
            double L01, double L02, double L03, string gmshRveFilePath, double[,] rveNodeData, List<int[,]> celltype2ELementsAndNodes, List<int[,]> celltype4ELementsAndNodes, int[] boundaryNodesIds1, List<int> rigidBodyConstraintNodeIds, double boundarySearchTol = 1e-09)
        {
            this.matrixMaterial = outterMaterial;
            this.inclusionMaterial = inclusionMaterial;
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


        }

        public IRVEbuilder Clone(int a)
        {
            return new GmshCompositeRveBuilder(matrixMaterial,inclusionMaterial,
                L01, L02, L03, gmshRveFilePath,  rveNodeData, celltype2ELementsAndNodes, celltype4ELementsAndNodes, boundaryNodesIds1, rigidBodyConstraintNodeIds, boundarySearchTol);
        }

        public Tuple<Model, Dictionary<int, INode>, double> GetModelAndBoundaryNodes()
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

                model.NodesDictionary.Add(nodeID, new Node(id: nodeID, x: nodeCoordX, y: nodeCoordY, z: nodeCoordZ));
            }


            int nElementGroupsI = celltype4ELementsAndNodes.Count;



            //define outer elements group
            int[] ContinuumTet4NodesNumbering = new int[4] { 0, 1, 2, 3 };
            int subdomainID = 0;
            var factoryOutter = new ContinuumElement3DFactory((ElasticMaterial3D)matrixMaterial, null);
            for (int i1 = 0; i1 < celltype4ELementsAndNodes[nElementGroupsI-1].GetLength(0); i1++)
            {
                List<Node> nodeSet = new List<Node>();
                for (int j = 0; j < 4; j++)
                {
                    int ren1 = ContinuumTet4NodesNumbering[j];
                    int nodeID = celltype4ELementsAndNodes[nElementGroupsI-1][i1, ren1+1];
                    nodeSet.Add((Node)model.NodesDictionary[nodeID]);
                }

				IElementType e1 = factoryOutter.CreateElement(CellType.Tet4, nodeSet);
				e1.ID = celltype4ELementsAndNodes[nElementGroupsI - 1][i1, 0];

				//Element e1 = new Element()
    //            {
    //                ID = celltype4ELementsAndNodes[nElementGroupsI-1][i1,0],
    //                ElementType = factoryOutter.CreateElement(CellType.Tet4, nodeSet) // dixws to e. exoume sfalma enw sto beambuilding oxi//edw kaleitai me ena orisma to Hexa8
    //            };

                //for (int j = 0; j < 4; j++)
                //{
                //    int ren1 = ContinuumTet4NodesNumbering[j];
                //    int nodeID = celltype4ELementsAndNodes[nElementGroupsI-1][i1, ren1+1];
                //    e1.NodesDictionary.Add(nodeID, model.NodesDictionary[nodeID]);
                //}
                model.ElementsDictionary.Add(e1.ID, e1);
				model.SubdomainsDictionary[subdomainID].Elements.Add(e1);
            }

            //define inner elements group
            var factoryInner = new ContinuumElement3DFactory((ElasticMaterial3D)inclusionMaterial, null);
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

					IElementType e1 = factoryInner.CreateElement(CellType.Tet4, nodeSet);
					e1.ID = celltype4ELementsAndNodes[i2][i1, 0];

					//Element e1 = new Element()
     //               {
     //                   ID = celltype4ELementsAndNodes[i2][i1, 0],
     //                   ElementType = factoryInner.CreateElement(CellType.Tet4, nodeSet) // dixws to e. exoume sfalma enw sto beambuilding oxi//edw kaleitai me ena orisma to Hexa8
     //               };

                    //for (int j = 0; j < 4; j++)
                    //{
                    //    int ren1 = ContinuumTet4NodesNumbering[j];
                    //    int nodeID = celltype4ELementsAndNodes[i2][i1, ren1 + 1];
                    //    e1.NodesDictionary.Add(nodeID, model.NodesDictionary[nodeID]);
                    //}
                    model.ElementsDictionary.Add(e1.ID, e1);
                    model.SubdomainsDictionary[subdomainID].Elements.Add(e1);
                }

            }


            Dictionary<int, INode> boundaryNodes = new Dictionary<int, INode>();

            for (int i = 0; i < boundaryNodesIds1.GetLength(0); i++)
            {
                boundaryNodes.Add(boundaryNodesIds1[i], model.NodesDictionary[boundaryNodesIds1[i]]);
            }




            return  new Tuple<Model, Dictionary<int, INode>, double>(model, boundaryNodes, L01*L02*L03); 
        }

        public Dictionary<Node, IList<IStructuralDofType>> GetModelRigidBodyNodeConstraints(Model model)
        {

            Dictionary<Node, IList<IStructuralDofType>> RigidBodyNodeConstraints = new Dictionary<Node, IList<IStructuralDofType>>();
            var rigidNodes = RigidNodes;

            RigidBodyNodeConstraints.Add((Node)model.NodesDictionary[rigidNodes[0]], new List<IStructuralDofType>() { StructuralDof.TranslationX, StructuralDof.TranslationY, StructuralDof.TranslationZ });
            RigidBodyNodeConstraints.Add((Node)model.NodesDictionary[rigidNodes[1]], new List<IStructuralDofType>() { StructuralDof.TranslationX, StructuralDof.TranslationY, StructuralDof.TranslationZ });
            RigidBodyNodeConstraints.Add((Node)model.NodesDictionary[rigidNodes[2]], new List<IStructuralDofType>() { StructuralDof.TranslationX, StructuralDof.TranslationY, StructuralDof.TranslationZ });

            return RigidBodyNodeConstraints;
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

                if (Z < -0.5 * L01 + boundarySearchTol)
                {
                    isBoundaryNode1 = true;
                }

                if (Z > +0.5 * L01 - boundarySearchTol)
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
