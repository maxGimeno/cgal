#include <ostream>
#include "vtkInformationVector.h"
#include "vtkIsotropicRemeshingWithEdgesFilter.h"
#include "vtkObjectFactory.h"
#include "vtkDataSet.h"
#include "vtkPointData.h"
#include "vtkCellData.h"
#include "vtkPolyData.h"
#include "vtkMultiBlockDataSet.h"
#include "vtkPointSet.h"
#include "vtkCellArray.h"
#include "vtkCell.h"
#include "vtkInformationStringKey.h"
#include "vtkUnstructuredGridWriter.h"
#include "vtkSmartPointer.h"
#include "vtkInformation.h"
#include <CGAL/Surface_mesh.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Polygon_mesh_processing/remesh.h>
using namespace CGAL;
vtkStandardNewMacro(vtkIsotropicRemeshingWithEdgesFilter);

typedef Simple_cartesian<double>    K;
typedef Surface_mesh<K::Point_3>    SM;

//----------------------------------------------------------------------------
vtkIsotropicRemeshingWithEdgesFilter::vtkIsotropicRemeshingWithEdgesFilter()
{
   SetNumberOfInputPorts(2);
   SetNumberOfOutputPorts(2);
   BboxInfo = 0;
   Bbox = 0;
}

//----------------------------------------------------------------------------
typedef boost::property_map<SM, CGAL::vertex_point_t>::type       VPMap;
typedef boost::property_map_value<SM, CGAL::vertex_point_t>::type Point_3;
typedef boost::graph_traits<SM>::vertex_descriptor                vertex_descriptor;
typedef boost::graph_traits<SM>::edge_descriptor                  edge_descriptor;
typedef boost::graph_traits<SM>::face_descriptor                  face_descriptor;
typedef boost::graph_traits<SM>::halfedge_descriptor              halfedge_descriptor;
vtkIsotropicRemeshingWithEdgesFilter::~vtkIsotropicRemeshingWithEdgesFilter()
{
}
struct Is_constrained_map
{
  boost::unordered_set<edge_descriptor>* m_set_ptr;

  typedef boost::unordered_set<edge_descriptor>::key_type  key_type;
  typedef bool                                             value_type;
  typedef bool                                             reference;
  typedef boost::read_write_property_map_tag               category;

  Is_constrained_map()
    : m_set_ptr(NULL)
  {}
  Is_constrained_map(boost::unordered_set<edge_descriptor>* set_)
    : m_set_ptr(set_)
  {}
  friend bool get(const Is_constrained_map& map, const key_type& k)
  {
    CGAL_assertion(map.m_set_ptr != NULL);
    return map.m_set_ptr->count(k);
  }
  friend void put(Is_constrained_map& map, const key_type& k, const value_type b)
  {
    CGAL_assertion(map.m_set_ptr != NULL);
    if (b)  map.m_set_ptr->insert(k);
    else    map.m_set_ptr->erase(k);
  }
};
int vtkIsotropicRemeshingWithEdgesFilter::RequestData(
  vtkInformation *vtkNotUsed(request),
  vtkInformationVector **inputVector,
  vtkInformationVector *outputVector)
{
  // Get the input and output data objects.
  // get the info objects
  vtkInformation *inInfo = inputVector[0]->GetInformationObject(0);
  vtkInformation *inInfoEdges = inputVector[1]->GetInformationObject(0);
  vtkInformation *outInfo = outputVector->GetInformationObject(0);
  vtkInformation *outEdgeInfo = outputVector->GetInformationObject(1);
  // get the input and output
  vtkDataSet *input = vtkDataSet::SafeDownCast(
    inInfo->Get(vtkDataObject::DATA_OBJECT()));
  vtkPolyData *inputEdges = vtkPolyData::SafeDownCast(
        inInfoEdges->Get(vtkDataObject::DATA_OBJECT()));
  vtkSmartPointer<vtkPolyData> outputEdges = vtkPolyData::SafeDownCast(
        outEdgeInfo->Get(vtkDataObject::DATA_OBJECT()));
  vtkSmartPointer<vtkPolyData> output = vtkPolyData::SafeDownCast(
        outInfo->Get(vtkDataObject::DATA_OBJECT()));
  vtkSmartPointer<vtkPolyData> polydata =
    vtkSmartPointer<vtkPolyData>::New();
  if(input->GetDataObjectType() == VTK_UNSTRUCTURED_GRID)
  {
    this->UnstructuredGridExecute(input, polydata);
  }
  else
  {
    polydata = vtkPolyData::SafeDownCast(input);
  }

  /***********************************
   * Create a SM from the input mesh *
   ***********************************/
  SM sm;
  VPMap vpmap = get(CGAL::vertex_point, sm);

  // get nb of points and cells
  vtkIdType nb_points = polydata->GetNumberOfPoints();
  vtkIdType nb_cells = polydata->GetNumberOfCells();
  //extract points
  std::vector<vertex_descriptor> vertex_map(nb_points);
  for (vtkIdType i = 0; i<nb_points; ++i)
  {
    double coords[3];
    polydata->GetPoint(i, coords);

    vertex_descriptor v = add_vertex(sm);
    put(vpmap, v, K::Point_3(coords[0], coords[1], coords[2]));
    vertex_map[i]=v;
  }
  //extract cells
  for (vtkIdType i = 0; i<nb_cells; ++i)
  {
    vtkCell* cell_ptr = polydata->GetCell(i);

    vtkIdType nb_vertices = cell_ptr->GetNumberOfPoints();

    std::vector<vertex_descriptor> vr(nb_vertices);
    for (vtkIdType k=0; k<nb_vertices; ++k)
      vr[k]=vertex_map[cell_ptr->GetPointId(k)];
    CGAL::Euler::add_face(vr, sm);
  }
  std::vector<vertex_descriptor> isolated_vertices;

  for(SM::vertex_iterator vit = sm.vertices_begin();
      vit != sm.vertices_end();
      ++vit)
  {
    if(sm.is_isolated(*vit))
      isolated_vertices.push_back(*vit);
  }
  for (std::size_t i=0; i < isolated_vertices.size(); ++i)
    sm.remove_vertex(isolated_vertices[i]);

  if(!is_triangle_mesh(sm))
  {
    vtkErrorMacro("The input mesh must be triangulated ");
    return 0;
  }

  /*********************************************
   * Create a Property map from the input edges*
   *********************************************/
  struct Edge
  {
    K::Point_3 source;
    K::Point_3 target;
  };

  boost::unordered_set<edge_descriptor> protect_edges;
  std::vector<edge_descriptor> sm_edges;
  BOOST_FOREACH(edge_descriptor ed, sm.edges())
      sm_edges.push_back(ed);

  double edge_max_length = 0;
  // get nb of points and cells
  nb_points = inputEdges->GetNumberOfPoints();
  nb_cells = inputEdges->GetNumberOfCells();
  //extract points
  std::vector<K::Point_3> points_map(nb_points);
  for (vtkIdType i = 0; i<nb_points; ++i)
  {
    double coords[3];
    inputEdges->GetPoint(i, coords);
    points_map[i]=K::Point_3(coords[0],coords[1],coords[2]);
  }
  //extract cells
  for (vtkIdType i = 0; i<nb_cells; ++i)
  {
    vtkCell* cell_ptr = inputEdges->GetCell(i);

    vtkIdType nb_vertices = cell_ptr->GetNumberOfPoints();
    if(nb_vertices != 2)
    {
      continue;
    }
    Edge new_edge;
    new_edge.source=points_map[cell_ptr->GetPointId(0)];
    new_edge.target=points_map[cell_ptr->GetPointId(1)];
    int length = (new_edge.source.x() - new_edge.target.x()) * (new_edge.source.x() - new_edge.target.x())-
        (new_edge.source.y() - new_edge.target.y()) * (new_edge.source.y() - new_edge.target.y())-
        (new_edge.source.z() - new_edge.target.z()) * (new_edge.source.z() - new_edge.target.z());
    if(std::sqrt(length) > edge_max_length)
      edge_max_length = length;

    //find corresponding edge_descriptor in sm
    for(std::vector<edge_descriptor>::iterator eit =
        sm_edges.begin(); eit != sm_edges.end();
        ++eit)
    {
      vertex_descriptor ta(source(*eit, sm)), tb(target(*eit, sm));
      if((vpmap[ta] ==  new_edge.source && vpmap[tb] ==  new_edge.target)
         ||
         (vpmap[ta] ==  new_edge.target && vpmap[tb] ==  new_edge.source))
      {
        protect_edges.insert(*eit);
        sm_edges.erase(eit);
        break;
      }
    }
  }

  /*****************************
   * Apply Isotropic remeshing *
   *****************************/
  Is_constrained_map cm(&protect_edges);
  if(!Split)
  {
    if(Protect && edge_max_length > 4/3 * Length)
    {
      vtkWarningMacro("Edge protection ignored (Constraints length must be <4/3*edge_length)");
    }
    if(protect_edges.empty())
      vtkWarningMacro("No edges were found for protection." );
    Polygon_mesh_processing::isotropic_remeshing(sm.faces(),
                                                 Length,
                                                 sm,
                                                 CGAL::Polygon_mesh_processing::parameters::number_of_iterations(MainIterations)
                                                 .protect_constraints((Protect && edge_max_length < 4/3 * Length))
                                                 .edge_is_constrained_map(cm)
                                                 .number_of_relaxation_steps(SmoothIterations)
                                                 .relax_constraints(Smooth));
  }
  else
  {
    boost::unordered_set<face_descriptor> sm_faces;
    BOOST_FOREACH(face_descriptor fd, sm.faces())
        sm_faces.insert(fd);
    std::vector<edge_descriptor> edges;
    BOOST_FOREACH(edge_descriptor e, protect_edges)
    {
      if (sm_faces.find(face(halfedge(e, sm), sm))
          != sm_faces.end()
          || sm_faces.find(face(opposite(halfedge(e, sm), sm), sm))
          != sm_faces.end())
        edges.push_back(e);
    }
    BOOST_FOREACH(face_descriptor f, sm_faces)
    {
      BOOST_FOREACH(halfedge_descriptor he, halfedges_around_face(halfedge(f, sm), sm))
      {
        if (sm_faces.find(face(opposite(he, sm), sm))
            == sm_faces.end())
          edges.push_back(edge(he, sm));
      }
    }
    if (!edges.empty())
      CGAL::Polygon_mesh_processing::split_long_edges(
            edges
            , Length
            , sm
            , PMP::parameters::geom_traits(K())
            .edge_is_constrained_map(cm));
    else
      vtkWarningMacro("No selected or boundary edges to be split" );
  }


  /**********************************
   * Pass the SM data to the output *
   **********************************/
  vtkPoints* const vtk_points = vtkPoints::New();
  vtkCellArray* const vtk_cells = vtkCellArray::New();
  vtk_points->Allocate(sm.number_of_vertices());
  vtk_cells->Allocate(sm.number_of_faces());

  std::map<vertex_descriptor, vtkIdType> Vids;
  vtkIdType inum = 0;

  BOOST_FOREACH(vertex_descriptor v, vertices(sm))
  {
    const K::Point_3& p = get(vpmap, v);
    vtk_points->InsertNextPoint(CGAL::to_double(p.x()),
                                CGAL::to_double(p.y()),
                                CGAL::to_double(p.z()));
    Vids[v] = inum++;
  }
  BOOST_FOREACH(face_descriptor f, faces(sm))
  {
    vtkIdList* cell = vtkIdList::New();
    BOOST_FOREACH(halfedge_descriptor h,
                  halfedges_around_face(halfedge(f, sm), sm))
    {
      cell->InsertNextId(Vids[target(h, sm)]);
    }
    vtk_cells->InsertNextCell(cell);
    cell->Delete();
  }
  output->SetPoints(vtk_points);
  vtk_points->Delete();
  output->SetPolys(vtk_cells);
  vtk_cells->Delete();
  output->Squeeze();

  /*************************************
   * Pass the Edges data to the output *
   *************************************/
  boost::unordered_set<vertex_descriptor> new_edge_points;
  BOOST_FOREACH(edge_descriptor ed, protect_edges)
  {
    new_edge_points.insert(source(ed, sm));
    new_edge_points.insert(target(ed, sm));
  }
  vtkPoints* const edge_points = vtkPoints::New();
  vtkCellArray* const edge_cells = vtkCellArray::New();
  edge_points->Allocate(new_edge_points.size());
  edge_cells->Allocate(protect_edges.size());

  std::map<vertex_descriptor, vtkIdType> EVids;
  inum = 0;

  BOOST_FOREACH(vertex_descriptor v, new_edge_points)
  {
    const K::Point_3& p = get(vpmap, v);
    edge_points->InsertNextPoint(CGAL::to_double(p.x()),
                                 CGAL::to_double(p.y()),
                                 CGAL::to_double(p.z()));
    EVids[v] = inum++;
  }
  BOOST_FOREACH(edge_descriptor ed, protect_edges)
  {
    vtkIdList* cell = vtkIdList::New();
    cell->InsertNextId(EVids[target(ed, sm)]);
    cell->InsertNextId(EVids[source(ed, sm)]);
    edge_cells->InsertNextCell(cell);
    cell->Delete();
  }

  outputEdges->SetPoints(edge_points);
  edge_points->Delete();
  outputEdges->SetLines(edge_cells);
  edge_cells->Delete();
  outputEdges->Squeeze();
  return 1;
}

int vtkIsotropicRemeshingWithEdgesFilter::FillInputPortInformation(int port, vtkInformation *info)
{
  if(port == 0)
  {
    info->Set(vtkAlgorithm::INPUT_REQUIRED_DATA_TYPE(), "vtkDataSet");
  }
  else if(port == 1)
  {
    info->Set(vtkAlgorithm::INPUT_REQUIRED_DATA_TYPE(), "vtkPolyData");
  }
  return 1;
}
int vtkIsotropicRemeshingWithEdgesFilter::FillOutputPortInformation(int, vtkInformation *info)
{
  info->Set(vtkDataObject::DATA_TYPE_NAME(), "vtkPolyData");
  return 1;
}

int vtkIsotropicRemeshingWithEdgesFilter::RequestInformation(vtkInformation *,
                                                             vtkInformationVector **inputVector,
                                                             vtkInformationVector *)
{
  vtkInformation *inInfo = inputVector[0]->GetInformationObject(0);
  // get the input and output
  vtkDataSet *input = vtkDataSet::SafeDownCast(
    inInfo->Get(vtkDataObject::DATA_OBJECT()));
  double * bounds = input->GetBounds();
  char *outstring = new char[300];
  sprintf (outstring, "min(%f, %f, %f)\n max(%f, %f, %f)",bounds[0], bounds[2], bounds[4], bounds[1], bounds[3], bounds[5]);
  SetBboxInfo(outstring);
  delete [] outstring;
  double diagonal = std::sqrt(
        (bounds[0]-bounds[1]) * (bounds[0]-bounds[1]) +
      (bounds[2]-bounds[3]) * (bounds[2]-bounds[3]) +
      (bounds[4]-bounds[5]) * (bounds[4]-bounds[5])
      );
  SetLengthInfo(0.01*diagonal);
  return 1;
}
