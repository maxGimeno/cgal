#include <vtkSmartPointer.h>
#include <vtkXMLUnstructuredGridReader.h>
#include <vtkXMLPolyDataReader.h>
#include <vtkDataSetReader.h>
#include <vtkDataSet.h>
#include <vtkUnstructuredGrid.h>
#include <vtkPolyData.h>
#include <vtkPointData.h>
#include <vtkCellData.h>
#include <vtkFieldData.h>
#include <vtkCellTypes.h>
#include <vtksys/SystemTools.hxx>

#include "vtkIsotropicRemeshingWithEdgesFilter.h"
 
#include <map>

int main (int argc, char *argv[])
{  
  const char* filename1 = (argc > 1) ? argv[1] : "data/Cube_tr.vtk";
  const char* filename2 = (argc > 2) ? argv[2] : "data/Cube-edges.vtk";
  vtkSmartPointer<vtkDataSetReader> reader =
    vtkSmartPointer<vtkDataSetReader>::New();
  reader->SetFileName(filename1);
  reader->Update();
  reader->GetOutput()->Register(reader);
  vtkIsotropicRemeshingWithEdgesFilter* filter =
    vtkIsotropicRemeshingWithEdgesFilter::New();
  filter->SetInputConnection(0, reader->GetOutputPort());
  vtkSmartPointer<vtkDataSetReader> reader2 =
    vtkSmartPointer<vtkDataSetReader>::New();
  reader2->SetFileName(filename2);
  reader2->Update();
  reader2->GetOutput()->Register(reader);
  filter->SetInputConnection(1, reader2->GetOutputPort());
  filter->SetLength(0.086);
  filter->SetProtect(0);
  filter->SetSmooth(0);
  filter->SetSplit(0);
  filter->SetMainIterations(1);
  filter->Update();
  vtkPolyData * mesh = vtkPolyData::SafeDownCast(filter->GetOutput(0));
  vtkPolyData * edges = vtkPolyData::SafeDownCast(filter->GetOutput(1));
  std::cout<<"The output has "<<mesh->GetNumberOfPolys()<<" triangles and the new edges have "<<edges->GetNumberOfLines()<<" lines."<<std::endl;
 
  return EXIT_SUCCESS;
}
