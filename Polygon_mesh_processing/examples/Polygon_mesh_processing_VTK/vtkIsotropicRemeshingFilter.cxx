#include <ostream>
#include "vtkInformationVector.h"
#include "vtkIsotropicRemeshingFilter.h"
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
vtkStandardNewMacro(vtkIsotropicRemeshingFilter);

typedef Simple_cartesian<double>    K;
typedef Surface_mesh<K::Point_3>    SM;

//----------------------------------------------------------------------------
vtkIsotropicRemeshingFilter::vtkIsotropicRemeshingFilter()
{
}

//----------------------------------------------------------------------------
typedef typename boost::property_map<SM, CGAL::vertex_point_t>::type       VPMap;
typedef typename boost::property_map_value<SM, CGAL::vertex_point_t>::type Point_3;
typedef typename boost::graph_traits<SM>::vertex_descriptor                vertex_descriptor;
typedef typename boost::graph_traits<SM>::edge_descriptor                  edge_descriptor;
typedef typename boost::graph_traits<SM>::face_descriptor                  face_descriptor;
typedef typename boost::graph_traits<SM>::halfedge_descriptor              halfedge_descriptor;
vtkIsotropicRemeshingFilter::~vtkIsotropicRemeshingFilter()
{
}
int vtkIsotropicRemeshingFilter::RequestData(
  vtkInformation *vtkNotUsed(request),
  vtkInformationVector **inputVector,
  vtkInformationVector *outputVector)
{
  // Get the input and output data objects.
  // get the info objects
  vtkInformation *inInfo = inputVector[0]->GetInformationObject(0);
  vtkInformation *outInfo = outputVector->GetInformationObject(0);
  // get the input and output
  vtkMultiBlockDataSet *b_input =
      vtkMultiBlockDataSet::SafeDownCast(inInfo->Get(vtkDataObject::DATA_OBJECT()));
  vtkMultiBlockDataSet *b_output =
      vtkMultiBlockDataSet::SafeDownCast(outInfo->Get(vtkDataObject::DATA_OBJECT()));
  b_output->SetNumberOfBlocks(b_input->GetNumberOfBlocks());
  for(unsigned int current_block=0;
      current_block < b_input->GetNumberOfBlocks();
      ++current_block)
  {
    vtkDataSet *input = vtkDataSet::SafeDownCast(
          b_input->GetBlock(current_block));
    vtkSmartPointer<vtkPolyData> output;
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
    /*****************************
   * Apply Isotropic remeshing *
   *****************************/
      Polygon_mesh_processing::isotropic_remeshing(sm.faces(),
                                                   Length,
                                                   sm,
                                                   CGAL::Polygon_mesh_processing::parameters::number_of_iterations(MainIterations));

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
    output=vtkSmartPointer<vtkPolyData>::New();
    output->SetPoints(vtk_points);
    vtk_points->Delete();
    output->SetPolys(vtk_cells);
    vtk_cells->Delete();
    output->Squeeze();
    b_output->SetBlock(current_block, output.GetPointer());
    char integer_string[32];
    sprintf(integer_string, "%d", current_block);
    char o_name[sizeof("Mesh ")+32] ="Mesh " ;
    strcat(o_name, integer_string);
    b_output->GetMetaData((unsigned int)current_block)->Set(vtkCompositeDataSet::NAME(),o_name);
  }
  return 1;
}

int vtkIsotropicRemeshingFilter::FillInputPortInformation(int, vtkInformation *info)
{
    info->Set(vtkAlgorithm::INPUT_REQUIRED_DATA_TYPE(), "vtkMultiBlockDataSet");
  return 1;
}
int vtkIsotropicRemeshingFilter::FillOutputPortInformation(int, vtkInformation *info)
{
  info->Set(vtkDataObject::DATA_TYPE_NAME(), "vtkMultiBlockDataSet");
  return 1;
}
