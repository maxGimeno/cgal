#include <vtkSmartPointer.h>
#include <vtkXMLUnstructuredGridReader.h>
#include <vtkDataSetReader.h>
#include <vtkDataSet.h>
#include <vtkUnstructuredGrid.h>
#include <vtkPolyData.h>
#include <vtkPointData.h>
#include <vtkCellData.h>
#include <vtkFieldData.h>
#include <vtkCellTypes.h>
#include <vtksys/SystemTools.hxx>
#include <vtkMultiBlockDataSet.h>
#include <vtk3DS.h>
#include <vtkMultiBlockDataGroupFilter.h>
#include "vtkIsotropicRemeshingFilter.h"
 
#include <map>

int main (int argc, char *argv[])
{  
  const char* filename1 = (argc > 1) ? argv[1] : "data/Cube_tr.vtk";
  const char* filename2 = (argc > 2) ? argv[2] : "data/Sphere.vtu";
  vtkSmartPointer<vtkDataSetReader> reader =
    vtkSmartPointer<vtkDataSetReader>::New();
  reader->SetFileName(filename1);
  reader->Update();
  reader->GetOutput()->Register(reader);
  vtkIsotropicRemeshingFilter* filter =
    vtkIsotropicRemeshingFilter::New();
  vtkSmartPointer<vtkXMLUnstructuredGridReader> reader2 =
    vtkSmartPointer<vtkXMLUnstructuredGridReader>::New();
  reader2->SetFileName(filename2);
  reader2->Update();
  reader2->GetOutput()->Register(reader);

  vtkMultiBlockDataGroupFilter* merger =
      vtkMultiBlockDataGroupFilter::New();
  merger->AddInputConnection(reader->GetOutputPort());
  merger->AddInputConnection(reader2->GetOutputPort());
  merger->Update();

  filter->AddInputConnection(merger->GetOutputPort());
  filter->SetLength(200);
  filter->SetMainIterations(1);
  filter->Update();


  vtkMultiBlockDataSet* output = vtkMultiBlockDataSet::SafeDownCast(filter->GetOutputDataObject(0));
  vtkPolyData * mesh1 = vtkPolyData::SafeDownCast(output->GetBlock(0));
  vtkPolyData * mesh2 = vtkPolyData::SafeDownCast(output->GetBlock(1));
  int i1 = mesh1->GetNumberOfPolys();
  int i2 = mesh2->GetNumberOfPolys();
  std::cout<<"The two outputs have repectively "<<i1<<" and "<<i2<<" triangles."<<std::endl;
 
  return EXIT_SUCCESS;
}
