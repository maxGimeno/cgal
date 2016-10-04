#ifndef vtkIsotropicRemeshingWithEdgesFilter_h
#define vtkIsotropicRemeshingWithEdgesFilter_h


#include "vtkFiltersCoreModule.h" // For export macro
#include "vtkGeometryFilter.h"


class VTK_EXPORT vtkIsotropicRemeshingWithEdgesFilter : public vtkGeometryFilter
{
public:
  static vtkIsotropicRemeshingWithEdgesFilter* New();
  vtkTypeMacro(vtkIsotropicRemeshingWithEdgesFilter, vtkGeometryFilter);
  vtkSetMacro(Split, int);
  vtkGetMacro(Split, int);
  vtkBooleanMacro(Split, int);

  vtkSetMacro(Length, double);
  vtkGetMacro(Length, double);


  vtkSetMacro(MainIterations, int);
  vtkGetMacro(MainIterations, int);

  vtkSetMacro(SmoothIterations, int);
  vtkGetMacro(SmoothIterations, int);

  vtkSetMacro(Protect, int);
  vtkGetMacro(Protect, int);
  vtkBooleanMacro(Protect, int);

  vtkSetMacro(Smooth, int);
  vtkGetMacro(Smooth, int);
  vtkBooleanMacro(Smooth, int);

  int RequestData(vtkInformation*,
                  vtkInformationVector** inputVector,
                  vtkInformationVector* outputVector);
  int FillInputPortInformation(int port, vtkInformation *info);
  int FillOutputPortInformation(int, vtkInformation *info);
protected:
  vtkIsotropicRemeshingWithEdgesFilter();
  ~vtkIsotropicRemeshingWithEdgesFilter();
  int Split;
  double Length;
  int MainIterations;
  int SmoothIterations;
  int Protect;
  int Smooth;

private:
  vtkIsotropicRemeshingWithEdgesFilter(const vtkIsotropicRemeshingWithEdgesFilter&);
  void operator=(const vtkIsotropicRemeshingWithEdgesFilter&);
};

#endif

