#ifndef vtkIsotropicRemeshingFilter_h
#define vtkIsotropicRemeshingFilter_h


#include "vtkFiltersCoreModule.h" // For export macro
#include "vtkGeometryFilter.h"


class VTK_EXPORT vtkIsotropicRemeshingFilter : public vtkGeometryFilter
{
public:
  static vtkIsotropicRemeshingFilter* New();
  vtkTypeMacro(vtkIsotropicRemeshingFilter, vtkGeometryFilter);

  vtkSetMacro(Length, double);
  vtkGetMacro(Length, double);


  vtkSetMacro(MainIterations, int);
  vtkGetMacro(MainIterations, int);

  int RequestData(vtkInformation*,
                  vtkInformationVector** inputVector,
                  vtkInformationVector* outputVector);
  int FillInputPortInformation(int, vtkInformation *info);
  int FillOutputPortInformation(int, vtkInformation *info);

protected:
  vtkIsotropicRemeshingFilter();
  ~vtkIsotropicRemeshingFilter();

  double Length;
  int MainIterations;
  int DuplicatedEdges;

private:
  vtkIsotropicRemeshingFilter(const vtkIsotropicRemeshingFilter&);
  void operator=(const vtkIsotropicRemeshingFilter&);
};

#endif

