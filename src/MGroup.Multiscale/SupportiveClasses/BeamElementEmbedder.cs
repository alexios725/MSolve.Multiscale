using System.Collections.Generic;
using System.Linq;

using ISAAR.MSolve.FEM.Interfaces;

using MGroup.MSolve.Discretization;
using MGroup.MSolve.Discretization.Embedding;
using MGroup.MSolve.Discretization.Entities;

namespace ISAAR.MSolve.FEM.Embedding
{
	public class BeamElementEmbedder : ElementEmbedder
	{
		public BeamElementEmbedder(Model model, IElementType embeddedElement, IEmbeddedDOFInHostTransformationVector transformation)
			: base(embeddedElement, transformation)
		{
		}

		protected void CalculateTransformationMatrix()
		{
			//var e = (IEmbeddedBeamElement)(embeddedElement.ElementType);
			base.CalculateTransformationMatrix();
			//transformationMatrix = e.CalculateRotationMatrix().MultiplyRight(transformationMatrix,true);
		}
	}
}
