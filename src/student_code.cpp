#include "student_code.h"
#include "mutablePriorityQueue.h"

using namespace std;

namespace CGL
{
  void BezierCurve::evaluateStep()
  {
    // TODO Part 1.
    // Perform one step of the Bezier curve's evaluation at t using de Casteljau's algorithm for subdivision.
    // Store all of the intermediate control points into the 2D vector evaluatedLevels.
    int point = (int) evaluatedLevels.size();
    evaluatedLevels.push_back(*new(std::vector<Vector2D>));
    for (int i = 0; i < numControlPoints - point; i += 1) {
      evaluatedLevels[point].push_back(evaluatedLevels[point-1][i] + (evaluatedLevels[point-1][i+1] + (-1 * evaluatedLevels[point-1][i])) * t);
    }
     
  }


  Vector3D BezierPatch::evaluate(double u, double v) const
  {
    // TODO Part 2.
    // Evaluate the Bezier surface at parameters (u, v) through 2D de Casteljau subdivision.
    // (i.e. Unlike Part 1 where we performed one subdivision level per call to evaluateStep, this function
    // should apply de Casteljau's algorithm until it computes the final, evaluated point on the surface)
    std::vector<Vector3D> temp;
    for (int r = 0; r < controlPoints.size(); r += 1) {
      temp.push_back(evaluate1D(controlPoints[r], u));
    }
    return evaluate1D(temp, v);
  }

  Vector3D BezierPatch::evaluate1D(std::vector<Vector3D> points, double t) const
  {
    // TODO Part 2.
    // Optional helper function that you might find useful to implement as an abstraction when implementing BezierPatch::evaluate.
    // Given an array of 4 points that lie on a single curve, evaluates the Bezier curve at parameter t using 1D de Casteljau subdivision.
    std::vector< std::vector<Vector3D> > evaluatedLevels;
    evaluatedLevels.push_back(points);
    int numControlPoints = (int) points.size();
    for (int i = 1; i < numControlPoints; i += 1) {
      evaluatedLevels.push_back(*new(std::vector<Vector3D>));
      for (int j = 0; j < numControlPoints - i; j += 1) {
        evaluatedLevels[i].push_back(evaluatedLevels[i-1][j] + (evaluatedLevels[i-1][j+1] + (-1 * evaluatedLevels[i-1][j])) * t);
      }
    }
    return evaluatedLevels.back()[0];
  }



  Vector3D Vertex::normal( void ) const
  {
    // TODO Part 3.
    // TODO Returns an approximate unit normal at this vertex, computed by
    // TODO taking the area-weighted average of the normals of neighboring
    // TODO triangles, then normalizing.
    Vector3D n(0,0,0); // initialize a vector to store your normal sum
    HalfedgeCIter h = halfedge();
    HalfedgeCIter h_start = h;
    Vector3D center = this->position;
    Vector3D last;
    Vector3D curr = h->twin()->vertex()->position + (-1 * center);
    bool go = true;
    while (go || h != h_start) {
      go = false;
      h = h->twin();
      last = curr;
      curr = h->vertex()->position + (-1 * center);
      n = n + Vector3D(last.y*curr.z-last.z*curr.y, last.z*curr.x-last.x*curr.z, last.x*curr.y-last.y*curr.x);
      h = h->next();
    }

    return -1 * n.unit();
  }

  EdgeIter HalfedgeMesh::flipEdge( EdgeIter e0 )
  {
    // TODO Part 4.
    // TODO This method should flip the given edge and return an iterator to the flipped edge.
    if (!e0->isBoundary()) {
      HalfedgeIter h = e0->halfedge();
      HalfedgeIter ht = h->twin();
      HalfedgeIter nh = h->next();
      HalfedgeIter nht = ht->next();
      HalfedgeIter revh = nh->next();
      HalfedgeIter revht = nht->next();
      
      //set new half edges for faces and vertices
      h->face()->halfedge()= h;
      ht->face()->halfedge() = ht;
      nh->vertex()->halfedge() = nh;
      nht->vertex()->halfedge() = nht;
      revh->vertex()->halfedge() = revh;
      revht->vertex()->halfedge() = revht;
      
      //do pointer reassignments
      revh->face() = ht->face();
      revht->face() = h->face();
      h->vertex() = revh->vertex();
      ht->vertex() = revht->vertex();
      revh->next() = nht;
      revht->next() = nh;
      nh->next() = h;
      nht->next() = ht;
      h->next() = revht;
      ht->next() = revh;
    }
    return e0;
  }

  VertexIter HalfedgeMesh::splitEdge( EdgeIter e0 )
  {
    // TODO Part 5.
    // TODO This method should split the given edge and return an iterator to the newly inserted vertex.
    // TODO The halfedge of this vertex should point along the edge that was split, rather than the new edges.
  if (!e0->isBoundary()) {
    HalfedgeIter h = e0->halfedge();
    HalfedgeIter ht = h->twin();
    HalfedgeIter nh = h->next();
    HalfedgeIter nht = ht->next();
    HalfedgeIter revh = nh->next();
    HalfedgeIter revht = nht->next();
    
    //set new half edges for existing faces and vertices
    nh->vertex()->halfedge() = nh;
    nht->vertex()->halfedge() = nht;
    revh->vertex()->halfedge() = revh;
    revht->vertex()->halfedge() = revht;
    h->face()->halfedge() = revh;
    ht->face()->halfedge() = nht;
    
    //create new elements and set pointers for new elements.
    HalfedgeIter h1 = newHalfedge();
    HalfedgeIter h2 = newHalfedge();
    HalfedgeIter h3 = newHalfedge();
    HalfedgeIter h4 = newHalfedge();
    HalfedgeIter h5 = newHalfedge();
    HalfedgeIter h6 = newHalfedge();
    EdgeIter e1 = newEdge();
    EdgeIter e2 = newEdge();
    EdgeIter e3 = newEdge();
    FaceIter f1 = newFace();
    FaceIter f2 = newFace();
    VertexIter c = newVertex();
    
    h1->setNeighbors( revh,  h2, c,               e1, h->face());
    h2->setNeighbors( h3,    h1, revh->vertex(),  e1, f1);
    h3->setNeighbors( nh,    h4, c,               e2, f1);
    h4->setNeighbors( h5,    h3, nh->vertex(),    e2, f2);
    h5->setNeighbors( revht, h6, c,               e3, f2);
    h6->setNeighbors( ht,    h5, revht->vertex(), e3, ht->face());
    e1->halfedge() = h1;
    e2->halfedge() = h3;
    e3->halfedge() = h5;
    e1->isNew = true;
    e3->isNew = true;
    f1->halfedge() = nh;
    f2->halfedge() = revht;
    c->position = 0.5 * (h->vertex()->position + nh->vertex()->position);
    c->halfedge() = ht;
    c->isNew = true;
    c->newPosition = e0->newPosition;
    e0->newPosition = Vector3D();
    
    //correct pointers for old elements.
    h->next() = h1;
    ht->vertex() = c;
    nh->next() = h2;
    nh->face() = f1;
    nht->next() = h6;
    revht->next() = h4;
    revht->face() = f2;
    
    return c;
    }
    return e0->halfedge()->vertex();
  }



  void MeshResampler::upsample( HalfedgeMesh& mesh )
  {
    // TODO Part 6.
    // This routine should increase the number of triangles in the mesh using Loop subdivision.
    // Each vertex and edge of the original surface can be associated with a vertex in the new (subdivided) surface.
    // Therefore, our strategy for computing the subdivided vertex locations is to *first* compute the new positions
    // using the connectity of the original (coarse) mesh; navigating this mesh will be much easier than navigating
    // the new subdivided (fine) mesh, which has more elements to traverse. We will then assign vertex positions in
    // the new mesh based on the values we computed for the original mesh.
    // TODO Compute new positions for all the vertices in the input mesh, using the Loop subdivision rule,
    // TODO and store them in Vertex::newPosition. At this point, we also want to mark each vertex as being
    // TODO a vertex of the original mesh.
    static const double ref[] = {0.0, 0.0, 0.0, 3.0 / 16.0};
    std::vector<double> weights(ref, ref + sizeof(ref) / sizeof(double));
    
    //First pass. Iterate through vertices and set new to false and compute new positions.
    for (VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++ ) {
      v->isNew = false;
      int deg = (int) v->degree();
      if (deg >= weights.size()) {
        for (int i = weights.size(); i <= deg; i += 1) {
          weights.push_back(3.0 / (i * 8.0));
        }
      }
      double weight = weights[deg];
      Vector3D tmp(0.0, 0.0, 0.0);
      HalfedgeIter start = v->halfedge();
      HalfedgeIter h = start;
      do {
         h = h->twin();
         tmp += weight * h->vertex()->position;
         h = h->next();
      } while (h != start);
      v->newPosition = tmp + (1 - deg * weight) * v->position;
    }
    
    // TODO Next, compute the updated vertex positions associated with edges, and store it in Edge::newPosition.
    //Second pass. Iterate through all edges and calculate new position.
    for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++) {
      e->isNew = false;
      HalfedgeIter h = e->halfedge();
      HalfedgeIter ht = h->twin();
      e->newPosition = 0.125 * (3.0 * (h->vertex()->position + ht->vertex()->position) + h->next()->next()->vertex()->position + ht->next()->next()->vertex()->position);
    }
    
    // TODO Next, we're going to split every edge in the mesh, in any order.  For future
    // TODO reference, we're also going to store some information about which subdivided
    // TODO edges come from splitting an edge in the original mesh, and which edges are new,
    // TODO by setting the flat Edge::isNew.  Note that in this loop, we only want to iterate
    // TODO over edges of the original mesh---otherwise, we'll end up splitting edges that we
    // TODO just split (and the loop will never end!)
    //Third pass. Iterate through all edges and split them. Split will handle transfer of new position and updating new vertex.
    for (EdgeIter e = mesh.edgesBegin(); !e->isNew; e++) {
      mesh.splitEdge(e);
    }
    
    // TODO Now flip any new edge that connects an old and new vertex.
    //Fourth pass. Iterate through all edges and flip any that are new to old.
    for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++) {
      if (e->isNew && (e->halfedge()->vertex()->isNew ^ e->halfedge()->twin()->vertex()->isNew)) {
        mesh.flipEdge(e);
        e->isNew = false;
      }
    }

    // TODO Finally, copy the new vertex positions into final Vertex::position.
    //Fifth pass. Iterate through all vertices and update new positions. Delete old ones.
    for (VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++) {
      v->position = v->newPosition;
      v->newPosition = Vector3D();
    }  
  }

}
