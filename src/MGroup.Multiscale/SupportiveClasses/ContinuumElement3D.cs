using System;
using System.Collections.Generic;
using MGroup.Constitutive.Structural;
using MGroup.Constitutive.Structural.Continuum;
using MGroup.Constitutive.Structural.Transient;
using MGroup.MSolve.Numerics.Interpolation;
using MGroup.MSolve.Numerics.Interpolation.GaussPointExtrapolation;
using MGroup.MSolve.Numerics.Interpolation.Jacobians;
using MGroup.LinearAlgebra.Matrices;
using MGroup.MSolve.Discretization;
using MGroup.MSolve.Numerics.Integration.Quadratures;
using MGroup.MSolve.Discretization.BoundaryConditions;
using MGroup.MSolve.Discretization.Dofs;
using MGroup.MSolve.Discretization.Entities;
using MGroup.MSolve.Discretization.Meshes;
using MGroup.MSolve.Geometry.Coordinates;
using MGroup.LinearAlgebra.Vectors;
using MGroup.MSolve.Discretization.Embedding;
using System.Linq;
using MGroup.MSolve.DataStructures;

namespace MGroup.Multiscale.SupportiveClasses
{
	/// <summary>
	/// Represents a continuum finite element for 3D problems. Specific elements (e.g. Hexa8, Hexa20, ...) can be created using
	/// the appropriate <see cref="IIsoparametricInterpolation3D_OLD"/>, <see cref="IQuadrature3D"/> etc. strategies. 
	/// Authors: Dimitris Tsapetis
	/// </summary>
	public class ContinuumElement3D : IStructuralElementType, ICell<INode>, IEmbeddedHostElement
	{
		private readonly static IDofType[] nodalDOFTypes = new IDofType[]
		{
			StructuralDof.TranslationX, StructuralDof.TranslationY, StructuralDof.TranslationZ
		};

		private readonly IDofType[][] dofTypes;
		private ITransientAnalysisProperties dynamicProperties;
		private readonly IReadOnlyList<IContinuumMaterial3D> materialsAtGaussPoints;

		private double[][] strainsVec;
		private double[][] strainsVecLastConverged;
		private double[][] lastStresses;

		public ContinuumElement3D(IReadOnlyList<INode> nodes, IIsoparametricInterpolation3D interpolation,
			IQuadrature3D quadratureForStiffness, IQuadrature3D quadratureForMass,
			IGaussPointExtrapolation3D gaussPointExtrapolation,
			IReadOnlyList<IContinuumMaterial3D> materialsAtGaussPoints, ITransientAnalysisProperties dynamicProperties)
		{
			this.dynamicProperties = dynamicProperties;
			this.materialsAtGaussPoints = materialsAtGaussPoints;
			this.GaussPointExtrapolation = gaussPointExtrapolation;
			this.Nodes = nodes;
			this.Interpolation = interpolation;
			this.QuadratureForConsistentMass = quadratureForMass;
			this.QuadratureForStiffness = quadratureForStiffness;

			dofTypes = new IDofType[nodes.Count][];
			for (int i = 0; i < nodes.Count; i++)
			{
				dofTypes[i] = new IDofType[]
				{
					StructuralDof.TranslationX, StructuralDof.TranslationY, StructuralDof.TranslationZ
				};
			}

			strainsVec = new double[materialsAtGaussPoints.Count][];
			strainsVecLastConverged = new double[materialsAtGaussPoints.Count][];
			lastStresses = new double[materialsAtGaussPoints.Count][];
			for (int gpoint = 0; gpoint < materialsAtGaussPoints.Count; gpoint++)
			{
				strainsVec[gpoint] = new double[6];
				strainsVecLastConverged[gpoint] = new double[6];
				lastStresses[gpoint] = new double[6];
			}
		}

		public CellType CellType => Interpolation.CellType;
		public IElementDofEnumerator DofEnumerator { get; set; } = new GenericDofEnumerator();
		public ElementDimensions ElementDimensions => ElementDimensions.ThreeD;
		public IGaussPointExtrapolation3D GaussPointExtrapolation { get; }
		public IReadOnlyList<IReadOnlyList<IDofType>> GetElementDofTypes() => dofTypes;

		public int ID { get; set; }
		public int SubdomainID { get; set; }
		public IReadOnlyList<INode> Nodes { get; }
		public IIsoparametricInterpolation3D Interpolation { get; }

		//public bool ConstitutiveLawModified
		//{
		//	get
		//	{
		//		foreach (IContinuumMaterial3D material in materialsAtGaussPoints)
		//		{
		//			if (material.IsCurrentStateDifferent()) return true;
		//		}
		//		return false;
		//	}
		//}

		public IQuadrature3D QuadratureForConsistentMass { get; }
		public IQuadrature3D QuadratureForStiffness { get; }

		public Matrix BuildConsistentMassMatrix()
		{
			int numberOfDofs = 3 * Nodes.Count;
			var mass = Matrix.CreateZero(numberOfDofs, numberOfDofs);
			IReadOnlyList<double[]> shapeFunctions =
				Interpolation.EvaluateFunctionsAtGaussPoints(QuadratureForConsistentMass);
			IReadOnlyList<Matrix> shapeGradientsNatural =
				Interpolation.EvaluateNaturalGradientsAtGaussPoints(QuadratureForConsistentMass);

			for (int gp = 0; gp < QuadratureForConsistentMass.IntegrationPoints.Count; ++gp)
			{
				Matrix shapeFunctionMatrix = BuildShapeFunctionMatrix(shapeFunctions[gp]);
				Matrix partial = shapeFunctionMatrix.MultiplyRight(shapeFunctionMatrix, true, false);
				var jacobian = new IsoparametricJacobian3D(Nodes, shapeGradientsNatural[gp]);
				double dA = jacobian.DirectDeterminant * QuadratureForConsistentMass.IntegrationPoints[gp].Weight;
				mass.AxpyIntoThis(partial, dA);
			}
			mass.ScaleIntoThis(dynamicProperties.Density);
			return mass;
		}

		public Matrix BuildLumpedMassMatrix()
		{
			int numberOfDofs = 3 * Nodes.Count;
			var lumpedMass = Matrix.CreateZero(numberOfDofs, numberOfDofs);
			IReadOnlyList<Matrix> shapeGradientsNatural =
				Interpolation.EvaluateNaturalGradientsAtGaussPoints(QuadratureForConsistentMass);

			double area = 0;
			for (int gp = 0; gp < QuadratureForConsistentMass.IntegrationPoints.Count; ++gp)
			{
				var jacobian = new IsoparametricJacobian3D(Nodes, shapeGradientsNatural[gp]);
				area += jacobian.DirectDeterminant * QuadratureForConsistentMass.IntegrationPoints[gp].Weight;
			}

			double nodalMass = area * dynamicProperties.Density / Nodes.Count;
			for (int i = 0; i < numberOfDofs; i++) lumpedMass[i, i] = nodalMass;

			return lumpedMass;
		}

		public IMatrix BuildStiffnessMatrix()
		{
			int numberOfDofs = 3 * Nodes.Count;
			var stiffness = Matrix.CreateZero(numberOfDofs, numberOfDofs);
			IReadOnlyList<Matrix> shapeGradientsNatural =
				Interpolation.EvaluateNaturalGradientsAtGaussPoints(QuadratureForStiffness);

			for (int gp = 0; gp < QuadratureForStiffness.IntegrationPoints.Count; ++gp)
			{
				IMatrixView constitutive = materialsAtGaussPoints[gp].ConstitutiveMatrix;
				var jacobian = new IsoparametricJacobian3D(Nodes, shapeGradientsNatural[gp]);
				Matrix shapeGradientsCartesian =
					jacobian.TransformNaturalDerivativesToCartesian(shapeGradientsNatural[gp]);
				Matrix deformation = BuildDeformationMatrix(shapeGradientsCartesian);

				Matrix partial = deformation.ThisTransposeTimesOtherTimesThis(constitutive);
				double dA = jacobian.DirectDeterminant * QuadratureForStiffness.IntegrationPoints[gp].Weight;
				stiffness.AxpyIntoThis(partial, dA);
			}

			return DofEnumerator.GetTransformedMatrix(stiffness);
		}

		//public double[] CalculateAccelerationResponseIntegral(IElementType element, IList<MassAccelerationLoad> loads)
		//{
		//	int numberOfDofs = 3 * Nodes.Count;
		//	var accelerations = new double[numberOfDofs];
		//	IMatrix massMatrix = MassMatrix(element);

		//	foreach (var load in loads)
		//	{
		//		int index = 0;
		//		foreach (var nodalDOFTypes in dofTypes)
		//		{
		//			foreach (var dofType in nodalDOFTypes)
		//			{
		//				if (dofType == load.DOF) accelerations[index] += load.Amount;
		//				index++;
		//			}
		//		}
		//	}

		//	return massMatrix.Multiply(accelerations);
		//}

		public double[] CalculateResponseIntegral()
		{
			int numberOfDofs = 3 * Nodes.Count;
			var Forces = Vector.CreateZero(numberOfDofs);
			IReadOnlyList<Matrix> shapeGradientsNatural =
				Interpolation.EvaluateNaturalGradientsAtGaussPoints(QuadratureForStiffness);
			for (int gp = 0; gp < QuadratureForStiffness.IntegrationPoints.Count; ++gp)
			{
				Vector Stresses = Vector.CreateFromArray(lastStresses[gp]);
				var jacobian = new IsoparametricJacobian3D(Nodes, shapeGradientsNatural[gp]);
				Matrix shapeGradientsCartesian =
					jacobian.TransformNaturalDerivativesToCartesian(shapeGradientsNatural[gp]);
				Matrix deformation = BuildDeformationMatrix(shapeGradientsCartesian);
				Vector gpForces = deformation.Transpose() * (Stresses);
				double dA = jacobian.DirectDeterminant * QuadratureForStiffness.IntegrationPoints[gp].Weight;
				gpForces.ScaleIntoThis(dA);
				Forces.AddIntoThis(gpForces);
			}
			return Forces.CopyToArray();
		}

		public double[] CalculateResponseIntegralForLogging(double[] localDisplacements)
		{
			return CalculateResponseIntegral();
		}

		public Tuple<double[], double[]> CalculateResponse(double[] localDisplacements)
		{
			int numberOfDofs = 3 * Nodes.Count;
			var Forces = Vector.CreateZero(numberOfDofs);
			IReadOnlyList<Matrix> shapeGradientsNatural =
				Interpolation.EvaluateNaturalGradientsAtGaussPoints(QuadratureForStiffness);	
			//double[] strains = new double[6];
			//for (int gp = 0; gp < QuadratureForStiffness.IntegrationPoints.Count; ++gp)
			//{
			//	strains = new double[6];
			//	var jacobian = new IsoparametricJacobian3D(Nodes, shapeGradientsNatural[gp]);
			//	Matrix shapeGradientsCartesian =
			//		jacobian.TransformNaturalDerivativesToCartesian(shapeGradientsNatural[gp]);
			//	Matrix deformation = BuildDeformationMatrix(shapeGradientsCartesian);
			//	strains = deformation.Multiply(localDisplacements);
			//	materialsAtGaussPoints[gp].UpdateMaterial(strains);
			//}

			//double[] strains = new double[6];
			double[] strainsVecMinusLastConvergedValue = new double[6];
			for (int gpo = 0; gpo < QuadratureForStiffness.IntegrationPoints.Count; ++gpo)
			{
				//strainsVec[gpo] = new double[6];
				var jacobian = new IsoparametricJacobian3D(Nodes, shapeGradientsNatural[gpo]);
				Matrix shapeGradientsCartesian =
					jacobian.TransformNaturalDerivativesToCartesian(shapeGradientsNatural[gpo]);
				Matrix deformation = BuildDeformationMatrix(shapeGradientsCartesian);
				strainsVec[gpo] = deformation.Multiply(localDisplacements);
				strainsVecMinusLastConvergedValue = new double[6]
				{
					strainsVec[gpo][0] - strainsVecLastConverged[gpo][0],
					strainsVec[gpo][1] - strainsVecLastConverged[gpo][1],
					strainsVec[gpo][2] - strainsVecLastConverged[gpo][2],
					strainsVec[gpo][3] - strainsVecLastConverged[gpo][3],
					strainsVec[gpo][4] - strainsVecLastConverged[gpo][4],
					strainsVec[gpo][5] - strainsVecLastConverged[gpo][5],
				};
				lastStresses[gpo] = materialsAtGaussPoints[gpo].UpdateConstitutiveMatrixAndEvaluateResponse(strainsVecMinusLastConvergedValue);
				//To update with total strain simplY = materialsAtGaussPoints[npoint].UpdateMaterial(strainsVec[npoint]);
			}


			return new Tuple<double[], double[]>(strainsVec[materialsAtGaussPoints.Count-1], lastStresses[materialsAtGaussPoints.Count - 1]);
		}

		public double CalculateVolume()
		{
			//TODO: Linear elements can use the more efficient rules for volume of polygons. Therefore this method should be 
			//      delegated to the interpolation.
			//TODO: A different integration rule should be used for integrating constant functions. For linear elements there
			//      is only 1 Gauss point (most probably), therefore the computational cost could be the same as using the 
			//      polygonal formulas.
			double volume = 0.0;
			IReadOnlyList<Matrix> shapeGradientsNatural =
				Interpolation.EvaluateNaturalGradientsAtGaussPoints(QuadratureForStiffness);
			for (int gp = 0; gp < QuadratureForStiffness.IntegrationPoints.Count; ++gp)
			{
				var jacobian = new IsoparametricJacobian3D(Nodes, shapeGradientsNatural[gp]);
				volume += jacobian.DirectDeterminant * QuadratureForStiffness.IntegrationPoints[gp].Weight; //TODO: this is used by all methods that integrate. I should cache it.
			}
			return volume;
		}

		//public void ClearMaterialState()
		//{
		//	foreach (var material in materialsAtGaussPoints) material.ClearState();
		//}

		//public void ClearMaterialStresses()
		//{
		//	foreach (var material in materialsAtGaussPoints) material.ClearStresses();
		//}

		public IMatrix DampingMatrix()
		{
			IMatrix damping = BuildStiffnessMatrix();
			damping.ScaleIntoThis(dynamicProperties.RayleighCoeffStiffness);
			damping.AxpyIntoThis(MassMatrix(), dynamicProperties.RayleighCoeffMass);
			return damping;
		}

		/// <summary>
		/// Calculates the coordinates of the centroid of this element.
		/// </summary>
		public CartesianPoint FindCentroid()
			=> Interpolation.TransformNaturalToCartesian(Nodes, new NaturalPoint(0.0, 0.0, 0.0));


		public IMatrix MassMatrix()
		{
			return BuildLumpedMassMatrix();
		}

		//public void ResetConstitutiveLawModified()
		//{
		//	foreach (var material in materialsAtGaussPoints) material.ResetModified();
		//}

		public void SaveConstitutiveLawState()
		{
			for (int npoint = 0; npoint < materialsAtGaussPoints.Count; npoint++)
			{
				for (int i1 = 0; i1 < 6; i1++)
				{ strainsVecLastConverged[npoint][i1] = strainsVec[npoint][i1]; }
			}
			foreach (var m in materialsAtGaussPoints) m.CreateState();
		}

		public IMatrix StiffnessMatrix() => DofEnumerator.GetTransformedMatrix(BuildStiffnessMatrix());
		public IMatrix PhysicsMatrix()
		{
			return StiffnessMatrix();
		}
		public (IReadOnlyList<double[]> strains, IReadOnlyList<double[]> stresses)
			UpdateStrainStressesAtGaussPoints(double[] localDisplacements)
		{
			int numberOfGPs = QuadratureForStiffness.IntegrationPoints.Count;
			var strains = new double[numberOfGPs][];
			var stresses = new double[numberOfGPs][];
			IReadOnlyList<Matrix> shapeGradientsNatural =
				Interpolation.EvaluateNaturalGradientsAtGaussPoints(QuadratureForStiffness);

			for (int gp = 0; gp < numberOfGPs; gp++)
			{
				IMatrixView constitutive = materialsAtGaussPoints[gp].ConstitutiveMatrix;
				var jacobian = new IsoparametricJacobian3D(Nodes, shapeGradientsNatural[gp]);
				Matrix shapeGrandientsCartesian =
					jacobian.TransformNaturalDerivativesToCartesian(shapeGradientsNatural[gp]);
				Matrix deformation = BuildDeformationMatrix(shapeGrandientsCartesian);

				strains[gp] = deformation.Multiply(localDisplacements);
				stresses[gp] = constitutive.Multiply(strains[gp]);
			}

			return (strains, stresses);
		}

		/// <summary>
		/// Assembles the deformation matrix of a solid element.
		/// The calculation are based on <see cref="https://www.colorado.edu/engineering/CAS/courses.d/AFEM.d/AFEM.Ch08.d/AFEM.Ch08.pdf"/>
		/// paragraph 8.4, equation 8.7
		/// </summary>
		/// <param name="shapeGradients"></param>
		/// <returns></returns>
		private Matrix BuildDeformationMatrix(Matrix shapeGradientsCartesian)
		{
			var deformation = Matrix.CreateZero(6, 3 * Nodes.Count);
			for (int nodeIdx = 0; nodeIdx < Nodes.Count; nodeIdx++)
			{
				int col0 = 3 * nodeIdx;
				int col1 = 3 * nodeIdx + 1;
				int col2 = 3 * nodeIdx + 2;

				deformation[0, col0] = shapeGradientsCartesian[nodeIdx, 0];
				deformation[1, col1] = shapeGradientsCartesian[nodeIdx, 1];
				deformation[2, col2] = shapeGradientsCartesian[nodeIdx, 2];

				deformation[3, col0] = shapeGradientsCartesian[nodeIdx, 1];
				deformation[3, col1] = shapeGradientsCartesian[nodeIdx, 0];

				deformation[4, col1] = shapeGradientsCartesian[nodeIdx, 2];
				deformation[4, col2] = shapeGradientsCartesian[nodeIdx, 1];

				deformation[5, col0] = shapeGradientsCartesian[nodeIdx, 2];
				deformation[5, col2] = shapeGradientsCartesian[nodeIdx, 0];
			}

			return deformation;
		}

		/// <summary>
		/// The shape function matrix is 2-by-2n, where n = is the number of shape functions. Row 0 corresponds to dof X, while
		/// row 1 to dof Y, etc.
		/// </summary>
		private Matrix BuildShapeFunctionMatrix(double[] shapeFunctions)
		{
			var shapeFunctionMatrix = Matrix.CreateZero(3, 3 * shapeFunctions.Length);
			for (int i = 0; i < shapeFunctions.Length; i++)
			{
				shapeFunctionMatrix[0, 3 * i] = shapeFunctions[i];
				shapeFunctionMatrix[1, 2 * i + 1] = shapeFunctions[i];
				shapeFunctionMatrix[2, 3 * i + 2] = shapeFunctions[i];
			}
			return shapeFunctionMatrix;
		}

		public EmbeddedNode BuildHostElementEmbeddedNode(IElementType element, INode node,
			IEmbeddedDOFInHostTransformationVector transformation)
		{
			var points = GetNaturalCoordinates(element, (Node)node);
			if (points.Length == 0) return null;

			//element.EmbeddedNodes.Add(node);
			var embeddedNode = new EmbeddedNode(node, element, transformation.GetDependentDOFTypes);
			for (int i = 0; i < points.Length; i++) embeddedNode.Coordinates.Add(points[i]);
			return embeddedNode;
		}
		//public EmbeddedNode BuildHostElementEmbeddedNode(IElementType element, INode node,
		//	IEmbeddedDOFInHostTransformationVector transformation)
		//{
		//	var points = GetNaturalCoordinates(element, (Node)node);
		//	if (points.Length == 0) return null;

		//	//element.EmbeddedNodes.Add(node);
		//	var embeddedNode = new EmbeddedNode(node, element, (IList<IDofType>)transformation.GetDOFTypesOfHost((EmbeddedNode)node));
		//	for (int i = 0; i < points.Length; i++) embeddedNode.Coordinates.Add(points[i]);
		//	return embeddedNode;
		//}

		private double[] GetNaturalCoordinates(IElementType element, Node node)
		{
			double[] mins = new double[] { element.Nodes[0].X, element.Nodes[0].Y, element.Nodes[0].Z };
			double[] maxes = new double[] { element.Nodes[0].X, element.Nodes[0].Y, element.Nodes[0].Z };
			for (int i = 0; i < element.Nodes.Count; i++)
			{
				mins[0] = mins[0] > element.Nodes[i].X ? element.Nodes[i].X : mins[0];
				mins[1] = mins[1] > element.Nodes[i].Y ? element.Nodes[i].Y : mins[1];
				mins[2] = mins[2] > element.Nodes[i].Z ? element.Nodes[i].Z : mins[2];
				maxes[0] = maxes[0] < element.Nodes[i].X ? element.Nodes[i].X : maxes[0];
				maxes[1] = maxes[1] < element.Nodes[i].Y ? element.Nodes[i].Y : maxes[1];
				maxes[2] = maxes[2] < element.Nodes[i].Z ? element.Nodes[i].Z : maxes[2];
			}
			//return new double[] { (node.X - mins[0]) / ((maxes[0] - mins[0]) / 2) - 1,
			//    (node.Y - mins[1]) / ((maxes[1] - mins[1]) / 2) - 1,
			//    (node.Z - mins[2]) / ((maxes[2] - mins[2]) / 2) - 1 };

			bool maybeInsideElement = node.X <= maxes[0] && node.X >= mins[0] &&
				node.Y <= maxes[1] && node.Y >= mins[1] &&
				node.Z <= maxes[2] && node.Z >= mins[2];
			if (maybeInsideElement == false) return new double[0];

			const int jacobianSize = 3;
			const int maxIterations = 1000;
			const double tolerance = 1e-10;
			int iterations = 0;
			double deltaNaturalCoordinatesNormSquare = 100;
			double[] naturalCoordinates = new double[] { 0, 0, 0 };
			const double toleranceSquare = tolerance * tolerance;

			while (deltaNaturalCoordinatesNormSquare > toleranceSquare && iterations < maxIterations)
			{
				iterations++;
				//var shapeFunctions = CalcH8Shape(naturalCoordinates[0], naturalCoordinates[1], naturalCoordinates[2]);
				var shapeFunctions = Interpolation.EvaluateFunctionsAt(new NaturalPoint(naturalCoordinates));
				double[] coordinateDifferences = new double[] { 0, 0, 0 };
				for (int i = 0; i < shapeFunctions.Length; i++)
				{
					coordinateDifferences[0] += shapeFunctions[i] * element.Nodes[i].X;
					coordinateDifferences[1] += shapeFunctions[i] * element.Nodes[i].Y;
					coordinateDifferences[2] += shapeFunctions[i] * element.Nodes[i].Z;
				}
				coordinateDifferences[0] = node.X - coordinateDifferences[0];
				coordinateDifferences[1] = node.Y - coordinateDifferences[1];
				coordinateDifferences[2] = node.Z - coordinateDifferences[2];

				double[,] faXYZ = GetCoordinatesTranspose(element);
				//double[] nablaShapeFunctions = CalcH8NablaShape(naturalCoordinates[0], naturalCoordinates[1], naturalCoordinates[2]);
				var nablaShapeFunctions = Interpolation.EvaluateNaturalGradientsAt(new NaturalPoint(naturalCoordinates));
				//var inverseJacobian = CalcH8JDetJ(faXYZ, nablaShapeFunctions).Item2;
				var inverseJacobian = new IsoparametricJacobian3D(element.Nodes.ToArray(), nablaShapeFunctions).InverseMatrix;

				double[] deltaNaturalCoordinates = new double[] { 0, 0, 0 };
				for (int i = 0; i < jacobianSize; i++)
					for (int j = 0; j < jacobianSize; j++)
						deltaNaturalCoordinates[i] += inverseJacobian[j, i] * coordinateDifferences[j];
				for (int i = 0; i < 3; i++)
					naturalCoordinates[i] += deltaNaturalCoordinates[i];

				deltaNaturalCoordinatesNormSquare = 0;
				for (int i = 0; i < 3; i++)
					deltaNaturalCoordinatesNormSquare += deltaNaturalCoordinates[i] * deltaNaturalCoordinates[i];
				//deltaNaturalCoordinatesNormSquare = Math.Sqrt(deltaNaturalCoordinatesNormSquare);
			}

			return naturalCoordinates.Count(x => Math.Abs(x) - 1.0 > tolerance) > 0 ? new double[0] : naturalCoordinates;
		}

		//private double[] CalcH8Shape(double fXi, double fEta, double fZeta)
		//{
		//	const double fSqC125 = 0.5;
		//	double auxilliaryfXiP = (1.0 + fXi) * fSqC125;
		//	double auxilliaryfEtaP = (1.0 + fEta) * fSqC125;
		//	double auxilliaryfZetaP = (1.0 + fZeta) * fSqC125;
		//	double auxilliaryfXiM = (1.0 - fXi) * fSqC125;
		//	double auxilliaryfEtaM = (1.0 - fEta) * fSqC125;
		//	double auxilliaryfZetaM = (1.0 - fZeta) * fSqC125;

		//	double[] auxH8ShapeFunctiondata = new double[8]; // Warning: shape function data not in hexa8fixed order.

		//	auxH8ShapeFunctiondata[0] = auxilliaryfXiP * auxilliaryfEtaP * auxilliaryfZetaP;
		//	auxH8ShapeFunctiondata[1] = auxilliaryfXiM * auxilliaryfEtaP * auxilliaryfZetaP;
		//	auxH8ShapeFunctiondata[2] = auxilliaryfXiM * auxilliaryfEtaM * auxilliaryfZetaP;
		//	auxH8ShapeFunctiondata[3] = auxilliaryfXiP * auxilliaryfEtaM * auxilliaryfZetaP;
		//	auxH8ShapeFunctiondata[4] = auxilliaryfXiP * auxilliaryfEtaP * auxilliaryfZetaM;
		//	auxH8ShapeFunctiondata[5] = auxilliaryfXiM * auxilliaryfEtaP * auxilliaryfZetaM;
		//	auxH8ShapeFunctiondata[6] = auxilliaryfXiM * auxilliaryfEtaM * auxilliaryfZetaM;
		//	auxH8ShapeFunctiondata[7] = auxilliaryfXiP * auxilliaryfEtaM * auxilliaryfZetaM;
		//	return auxH8ShapeFunctiondata;
		//}

		protected double[,] GetCoordinatesTranspose(IElementType element)
		{
			double[,] nodeCoordinatesXYZ = new double[3, dofTypes.Length];
			for (int i = 0; i < dofTypes.Length; i++)
			{
				nodeCoordinatesXYZ[0, i] = element.Nodes[i].X;
				nodeCoordinatesXYZ[1, i] = element.Nodes[i].Y;
				nodeCoordinatesXYZ[2, i] = element.Nodes[i].Z;
			}
			return nodeCoordinatesXYZ;
		}

		//private double[] CalcH8NablaShape(double fXi, double fEta, double fZeta)
		//{
		//	const double fSq125 = 0.35355339059327376220042218105242;
		//	double[] auxilliaryfaDS = new double[24];

		//	double auxilliaryfXiP = (1.0 + fXi) * fSq125;
		//	double auxilliaryfEtaP = (1.0 + fEta) * fSq125;
		//	double auxilliaryfZetaP = (1.0 + fZeta) * fSq125;
		//	double auxilliaryfXiM = (1.0 - fXi) * fSq125;
		//	double auxilliaryfEtaM = (1.0 - fEta) * fSq125;
		//	double auxilliaryfZetaM = (1.0 - fZeta) * fSq125;

		//	auxilliaryfaDS[0] = auxilliaryfEtaP * auxilliaryfZetaP;
		//	auxilliaryfaDS[1] = -auxilliaryfEtaP * auxilliaryfZetaP;
		//	auxilliaryfaDS[2] = -auxilliaryfEtaM * auxilliaryfZetaP;
		//	auxilliaryfaDS[3] = auxilliaryfEtaM * auxilliaryfZetaP;
		//	auxilliaryfaDS[4] = auxilliaryfEtaP * auxilliaryfZetaM;
		//	auxilliaryfaDS[5] = -auxilliaryfEtaP * auxilliaryfZetaM;
		//	auxilliaryfaDS[6] = -auxilliaryfEtaM * auxilliaryfZetaM;
		//	auxilliaryfaDS[7] = auxilliaryfEtaM * auxilliaryfZetaM;

		//	auxilliaryfaDS[8] = auxilliaryfXiP * auxilliaryfZetaP;
		//	auxilliaryfaDS[9] = auxilliaryfXiM * auxilliaryfZetaP;
		//	auxilliaryfaDS[10] = -auxilliaryfXiM * auxilliaryfZetaP;
		//	auxilliaryfaDS[11] = -auxilliaryfXiP * auxilliaryfZetaP;
		//	auxilliaryfaDS[12] = auxilliaryfXiP * auxilliaryfZetaM;
		//	auxilliaryfaDS[13] = auxilliaryfXiM * auxilliaryfZetaM;
		//	auxilliaryfaDS[14] = -auxilliaryfXiM * auxilliaryfZetaM;
		//	auxilliaryfaDS[15] = -auxilliaryfXiP * auxilliaryfZetaM;

		//	auxilliaryfaDS[16] = auxilliaryfXiP * auxilliaryfEtaP;
		//	auxilliaryfaDS[17] = auxilliaryfXiM * auxilliaryfEtaP;
		//	auxilliaryfaDS[18] = auxilliaryfXiM * auxilliaryfEtaM;
		//	auxilliaryfaDS[19] = auxilliaryfXiP * auxilliaryfEtaM;
		//	auxilliaryfaDS[20] = -auxilliaryfXiP * auxilliaryfEtaP;
		//	auxilliaryfaDS[21] = -auxilliaryfXiM * auxilliaryfEtaP;
		//	auxilliaryfaDS[22] = -auxilliaryfXiM * auxilliaryfEtaM;
		//	auxilliaryfaDS[23] = -auxilliaryfXiP * auxilliaryfEtaM;

		//	return auxilliaryfaDS;
		//}

		protected static double determinantTolerance = 0.00000001;
		//private Tuple<double[,], double[,], double> CalcH8JDetJ(double[,] faXYZ, double[] faDS)
		//{
		//	double[,] auxilliaryfaJ = new double[3, 3];
		//	auxilliaryfaJ[0, 0] = faDS[0] * faXYZ[0, 0] + faDS[1] * faXYZ[0, 1] + faDS[2] * faXYZ[0, 2] + faDS[3] * faXYZ[0, 3] + faDS[4] * faXYZ[0, 4] + faDS[5] * faXYZ[0, 5] + faDS[6] * faXYZ[0, 6] + faDS[7] * faXYZ[0, 7];
		//	auxilliaryfaJ[0, 1] = faDS[0] * faXYZ[1, 0] + faDS[1] * faXYZ[1, 1] + faDS[2] * faXYZ[1, 2] + faDS[3] * faXYZ[1, 3] + faDS[4] * faXYZ[1, 4] + faDS[5] * faXYZ[1, 5] + faDS[6] * faXYZ[1, 6] + faDS[7] * faXYZ[1, 7];
		//	auxilliaryfaJ[0, 2] = faDS[0] * faXYZ[2, 0] + faDS[1] * faXYZ[2, 1] + faDS[2] * faXYZ[2, 2] + faDS[3] * faXYZ[2, 3] + faDS[4] * faXYZ[2, 4] + faDS[5] * faXYZ[2, 5] + faDS[6] * faXYZ[2, 6] + faDS[7] * faXYZ[2, 7];
		//	auxilliaryfaJ[1, 0] = faDS[8] * faXYZ[0, 0] + faDS[9] * faXYZ[0, 1] + faDS[10] * faXYZ[0, 2] + faDS[11] * faXYZ[0, 3] + faDS[12] * faXYZ[0, 4] + faDS[13] * faXYZ[0, 5] + faDS[14] * faXYZ[0, 6] + faDS[15] * faXYZ[0, 7];
		//	auxilliaryfaJ[1, 1] = faDS[8] * faXYZ[1, 0] + faDS[9] * faXYZ[1, 1] + faDS[10] * faXYZ[1, 2] + faDS[11] * faXYZ[1, 3] + faDS[12] * faXYZ[1, 4] + faDS[13] * faXYZ[1, 5] + faDS[14] * faXYZ[1, 6] + faDS[15] * faXYZ[1, 7];
		//	auxilliaryfaJ[1, 2] = faDS[8] * faXYZ[2, 0] + faDS[9] * faXYZ[2, 1] + faDS[10] * faXYZ[2, 2] + faDS[11] * faXYZ[2, 3] + faDS[12] * faXYZ[2, 4] + faDS[13] * faXYZ[2, 5] + faDS[14] * faXYZ[2, 6] + faDS[15] * faXYZ[2, 7];
		//	auxilliaryfaJ[2, 0] = faDS[16] * faXYZ[0, 0] + faDS[17] * faXYZ[0, 1] + faDS[18] * faXYZ[0, 2] + faDS[19] * faXYZ[0, 3] + faDS[20] * faXYZ[0, 4] + faDS[21] * faXYZ[0, 5] + faDS[22] * faXYZ[0, 6] + faDS[23] * faXYZ[0, 7];
		//	auxilliaryfaJ[2, 1] = faDS[16] * faXYZ[1, 0] + faDS[17] * faXYZ[1, 1] + faDS[18] * faXYZ[1, 2] + faDS[19] * faXYZ[1, 3] + faDS[20] * faXYZ[1, 4] + faDS[21] * faXYZ[1, 5] + faDS[22] * faXYZ[1, 6] + faDS[23] * faXYZ[1, 7];
		//	auxilliaryfaJ[2, 2] = faDS[16] * faXYZ[2, 0] + faDS[17] * faXYZ[2, 1] + faDS[18] * faXYZ[2, 2] + faDS[19] * faXYZ[2, 3] + faDS[20] * faXYZ[2, 4] + faDS[21] * faXYZ[2, 5] + faDS[22] * faXYZ[2, 6] + faDS[23] * faXYZ[2, 7];

		//	double auxilliaryfDet1 = auxilliaryfaJ[0, 0] * (auxilliaryfaJ[1, 1] * auxilliaryfaJ[2, 2] - auxilliaryfaJ[2, 1] * auxilliaryfaJ[1, 2]);
		//	double auxilliaryfDet2 = -auxilliaryfaJ[0, 1] * (auxilliaryfaJ[1, 0] * auxilliaryfaJ[2, 2] - auxilliaryfaJ[2, 0] * auxilliaryfaJ[1, 2]);
		//	double auxilliaryfDet3 = auxilliaryfaJ[0, 2] * (auxilliaryfaJ[1, 0] * auxilliaryfaJ[2, 1] - auxilliaryfaJ[2, 0] * auxilliaryfaJ[1, 1]);
		//	double auxilliaryfDetJ = auxilliaryfDet1 + auxilliaryfDet2 + auxilliaryfDet3;
		//	if (auxilliaryfDetJ < determinantTolerance)
		//	{
		//		throw new ArgumentException(
		//			$"Jacobian determinant is negative or under tolerance ({auxilliaryfDetJ} < {determinantTolerance})."
		//			 + " Check the order of nodes or the element geometry.");
		//	}

		//	double auxilliaryfDetInv = 1.0 / auxilliaryfDetJ;
		//	double[,] auxilliaryfaJInv = new double[3, 3];
		//	auxilliaryfaJInv[0, 0] = (auxilliaryfaJ[1, 1] * auxilliaryfaJ[2, 2] - auxilliaryfaJ[2, 1] * auxilliaryfaJ[1, 2]) * auxilliaryfDetInv;
		//	auxilliaryfaJInv[1, 0] = (auxilliaryfaJ[2, 0] * auxilliaryfaJ[1, 2] - auxilliaryfaJ[1, 0] * auxilliaryfaJ[2, 2]) * auxilliaryfDetInv;
		//	auxilliaryfaJInv[2, 0] = (auxilliaryfaJ[1, 0] * auxilliaryfaJ[2, 1] - auxilliaryfaJ[2, 0] * auxilliaryfaJ[1, 1]) * auxilliaryfDetInv;
		//	auxilliaryfaJInv[0, 1] = (auxilliaryfaJ[2, 1] * auxilliaryfaJ[0, 2] - auxilliaryfaJ[0, 1] * auxilliaryfaJ[2, 2]) * auxilliaryfDetInv;
		//	auxilliaryfaJInv[1, 1] = (auxilliaryfaJ[0, 0] * auxilliaryfaJ[2, 2] - auxilliaryfaJ[2, 0] * auxilliaryfaJ[0, 2]) * auxilliaryfDetInv;
		//	auxilliaryfaJInv[2, 1] = (auxilliaryfaJ[2, 0] * auxilliaryfaJ[0, 1] - auxilliaryfaJ[2, 1] * auxilliaryfaJ[0, 0]) * auxilliaryfDetInv;
		//	auxilliaryfaJInv[0, 2] = (auxilliaryfaJ[0, 1] * auxilliaryfaJ[1, 2] - auxilliaryfaJ[1, 1] * auxilliaryfaJ[0, 2]) * auxilliaryfDetInv;
		//	auxilliaryfaJInv[1, 2] = (auxilliaryfaJ[1, 0] * auxilliaryfaJ[0, 2] - auxilliaryfaJ[0, 0] * auxilliaryfaJ[1, 2]) * auxilliaryfDetInv;
		//	auxilliaryfaJInv[2, 2] = (auxilliaryfaJ[0, 0] * auxilliaryfaJ[1, 1] - auxilliaryfaJ[1, 0] * auxilliaryfaJ[0, 1]) * auxilliaryfDetInv;

		//	return new Tuple<double[,], double[,], double>(auxilliaryfaJ, auxilliaryfaJInv, auxilliaryfDetJ);
		//}

		//double[] IEmbeddedHostElement.GetShapeFunctionsForNode(IElementType element, EmbeddedNode node)
		//{
		//	double[,] elementCoordinates = GetCoordinatesTranspose(element);
		//	var shapeFunctions = CalcH8Shape(node.Coordinates[0], node.Coordinates[1], node.Coordinates[2]);
		//	var nablaShapeFunctions = CalcH8NablaShape(node.Coordinates[0], node.Coordinates[1], node.Coordinates[2]);
		//	var jacobian = CalcH8JDetJ(elementCoordinates, nablaShapeFunctions);

		//	return new double[]
		//	{
		//		shapeFunctions[0], shapeFunctions[1], shapeFunctions[2], shapeFunctions[3], shapeFunctions[4], shapeFunctions[5], shapeFunctions[6], shapeFunctions[7],
		//		nablaShapeFunctions[0], nablaShapeFunctions[1], nablaShapeFunctions[2], nablaShapeFunctions[3], nablaShapeFunctions[4], nablaShapeFunctions[5], nablaShapeFunctions[6], nablaShapeFunctions[7],
		//		nablaShapeFunctions[8], nablaShapeFunctions[9], nablaShapeFunctions[10], nablaShapeFunctions[11], nablaShapeFunctions[12], nablaShapeFunctions[13], nablaShapeFunctions[14], nablaShapeFunctions[15],
		//		nablaShapeFunctions[16], nablaShapeFunctions[17], nablaShapeFunctions[18], nablaShapeFunctions[19], nablaShapeFunctions[20], nablaShapeFunctions[21], nablaShapeFunctions[22], nablaShapeFunctions[23],
		//		jacobian.Item1[0, 0], jacobian.Item1[0, 1], jacobian.Item1[0, 2], jacobian.Item1[1, 0], jacobian.Item1[1, 1], jacobian.Item1[1, 2], jacobian.Item1[2, 0], jacobian.Item1[2, 1], jacobian.Item1[2, 2],
		//		jacobian.Item2[0, 0], jacobian.Item2[0, 1], jacobian.Item2[0, 2], jacobian.Item2[1, 0], jacobian.Item2[1, 1], jacobian.Item2[1, 2], jacobian.Item2[2, 0], jacobian.Item2[2, 1], jacobian.Item2[2, 2]
		//	};
		//}

		double[] IEmbeddedHostElement.GetShapeFunctionsForNode(IElementType element, EmbeddedNode node)
		{
			var naturalPoint = new NaturalPoint(node.Coordinates.ToArray());
			var shapeFunctions = Interpolation.EvaluateFunctionsAt(naturalPoint);
			var shapeFunctionGradients = Interpolation.EvaluateNaturalGradientsAt(naturalPoint);
			var jacobian = new IsoparametricJacobian3D(element.Nodes.ToArray(), shapeFunctionGradients);

			var shapeFunctionGradients2DArray = shapeFunctionGradients.CopyToArray2D();
			var shapeFunctionGradients1DArray = new double[shapeFunctionGradients2DArray.GetLength(0) * shapeFunctionGradients2DArray.GetLength(1)];
			for (int j = 0; j < shapeFunctionGradients2DArray.GetLength(1); j++)
			{
				for (int i = 0; i < shapeFunctionGradients2DArray.GetLength(0); i++)
				{
					shapeFunctionGradients1DArray[(shapeFunctionGradients2DArray.GetLength(0) * j) + i] = shapeFunctionGradients2DArray[i, j];
				}
			}

			var jacobianDirect2DArray = jacobian.DirectMatrix.CopyToArray2D();
			var jacobianInverse2DArray = jacobian.InverseMatrix.CopyToArray2D();
			var jacobianDirect1DArray = new double[jacobianDirect2DArray.GetLength(0) * jacobianDirect2DArray.GetLength(1)];
			var jacobianInverse1DArray = new double[jacobianInverse2DArray.GetLength(0) * jacobianInverse2DArray.GetLength(1)];
			for (int i = 0; i < jacobianDirect2DArray.GetLength(0); i++)
			{
				for (int j = 0; j < jacobianDirect2DArray.GetLength(1); j++)
				{
					jacobianDirect1DArray[(jacobianDirect2DArray.GetLength(1) * i) + j] = jacobianDirect2DArray[i, j];
					jacobianInverse1DArray[(jacobianInverse2DArray.GetLength(1) * i) + j] = jacobianInverse2DArray[i, j];
				}
			}
			var returnValueList = new List<double>();
			foreach (double shapeFunction in shapeFunctions)
			{
				returnValueList.Add(shapeFunction);
			}
			foreach (double shapeFunctionGradient in shapeFunctionGradients1DArray)
			{
				returnValueList.Add(shapeFunctionGradient);
			}
			foreach (double value in jacobianDirect1DArray)
			{
				returnValueList.Add(value);
			}
			foreach (double value in jacobianInverse1DArray)
			{
				returnValueList.Add(value);
			}

			return returnValueList.ToArray();
		}

		public void SaveConstitutiveLawState(IHaveState externalState) => SaveConstitutiveLawState();

		//public (double[] shapeFunctions, Matrix shapeFunctionGradients, IsoparametricJacobian3D jacobian) GetShapeFunctionsForNode(IElementType element, EmbeddedNode node)
		//{
		//	var naturalPoint = new NaturalPoint(node.Coordinates.ToArray());
		//	var shapeFunctions = Interpolation.EvaluateFunctionsAt(naturalPoint);
		//	var shapeFunctionGradients = Interpolation.EvaluateNaturalGradientsAt(naturalPoint);
		//	var jacobian = new IsoparametricJacobian3D(element.Nodes.ToArray(), shapeFunctionGradients);
		//	var shapeFunctionGradients2DArray = shapeFunctionGradients.CopyToArray2D();

		//	var jacobianDirect = jacobian.DirectMatrix;
		//	var jacobianInverse = jacobian.InverseMatrix;
		//	var jacobianDirect2DArray = jacobianDirect.CopyToArray2D();
		//	var jacobianInverse2DArray = jacobianInverse.CopyToArray2D();


		//	return (shapeFunctions, shapeFunctionGradients, jacobian);
		//}
	}
}
