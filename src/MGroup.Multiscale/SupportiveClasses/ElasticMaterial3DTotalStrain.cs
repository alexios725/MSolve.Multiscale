//using System;

//using MGroup.Constitutive.Structural.Continuum;
//using MGroup.LinearAlgebra.Matrices;

//namespace MGroup.Multiscale.SupportiveClasses
//{
//	public class ElasticMaterial3DTotalStrain : IIsotropicContinuumMaterial3D
//	{
//		private Matrix constitutiveMatrix = null;
//		public double YoungModulus { get; set; }
//		public double PoissonRatio { get; set; }
//		public double[] Coordinates { get; set; }
//		private double[] stressesNew = new double[6];

//		public ElasticMaterial3DTotalStrain(double poissonsRation, double youngsModoulus)
//		{
//			constitutiveMatrix = GetConstitutiveMatrix(youngsModoulus, poissonsRation);
//		}

//		public ElasticMaterial3DTotalStrain(Matrix constitutiveMatrix)
//		{
//			this.constitutiveMatrix = constitutiveMatrix;
//		}

//		private Matrix GetConstitutiveMatrix(double youngsModoulus, double poissonsRation)
//		{
//			double fE1 = youngsModoulus / (double)(1 + poissonsRation);
//			double fE2 = fE1 * poissonsRation / (double)(1 - 2 * poissonsRation);
//			double fE3 = fE1 + fE2;
//			double fE4 = fE1 * 0.5;
//			var afE = Matrix.CreateZero(6, 6);
//			afE[0, 0] = fE3;
//			afE[0, 1] = fE2;
//			afE[0, 2] = fE2;
//			afE[1, 0] = fE2;
//			afE[1, 1] = fE3;
//			afE[1, 2] = fE2;
//			afE[2, 0] = fE2;
//			afE[2, 1] = fE2;
//			afE[2, 2] = fE3;
//			afE[3, 3] = fE4;
//			afE[4, 4] = fE4;
//			afE[5, 5] = fE4;

//			return afE;
//		}

//		private void CalculateNextStressStrainPoint(double[] totalStrains)
//		{
//			stressesNew = new double[6];
//			for (int i = 0; i < 6; i++)
//			{
//				for (int j = 0; j < 6; j++)
//				{ stressesNew[i] += constitutiveMatrix[i, j] * totalStrains[j]; }
//			}

//		}

//		#region IFiniteElementMaterial Members

//		public int ID { get; set; }

//		public bool Modified => false;

//		public void ResetModified() { }

//		#endregion

//		#region IFiniteElementMaterial3D Members

//		public double[] Stresses => stressesNew;

//		public IMatrixView ConstitutiveMatrix
//		{
//			get
//			{
//				if (constitutiveMatrix == null) UpdateMaterial(new double[6]);
//				return constitutiveMatrix;
//			}
//		}

//		public void UpdateMaterial(double[] totalStrains)
//		{
//			throw new NotImplementedException();
//			CalculateNextStressStrainPoint(totalStrains);

//		}

//		public void ClearState()
//		{
//			stressesNew.Clear();
//		}

//		public void SaveState() { }

//		public void ClearStresses()
//		{
//			stressesNew.Clear();
//		}

//		#endregion

//		#region ICloneable Members

//		object ICloneable.Clone() => Clone();

//		public ElasticMaterial3DTotalStrain Clone()
//		{
//			return new ElasticMaterial3DTotalStrain(constitutiveMatrix);
//		}

//		#endregion

//	}

//}
