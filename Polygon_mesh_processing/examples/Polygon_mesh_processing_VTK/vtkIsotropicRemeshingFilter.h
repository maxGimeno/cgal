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

  vtkSetMacro(LengthInfo, double);
  vtkGetMacro(LengthInfo, double);

  vtkSetStringMacro(BboxInfo);
  vtkGetStringMacro(BboxInfo);

  vtkSetStringMacro(Bbox);
  vtkGetStringMacro(Bbox);

  vtkSetMacro(MainIterations, int);
  vtkGetMacro(MainIterations, int);

  int RequestData(vtkInformation*,
                  vtkInformationVector** inputVector,
                  vtkInformationVector* outputVector);

  int RequestInformation(vtkInformation *,
                         vtkInformationVector **inputVector,
                         vtkInformationVector *outputVector);
  int FillInputPortInformation(int, vtkInformation *info);
  int FillOutputPortInformation(int, vtkInformation *info);

protected:
  vtkIsotropicRemeshingFilter();
  ~vtkIsotropicRemeshingFilter();

  double Length;
  double LengthInfo;
  int MainIterations;
  char* BboxInfo;
  char* Bbox;

private:
  vtkIsotropicRemeshingFilter(const vtkIsotropicRemeshingFilter&);
  void operator=(const vtkIsotropicRemeshingFilter&);
};

#endif

