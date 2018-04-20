#undef QT_NO_KEYWORDS
#include <CGAL/Three/Polyhedron_demo_plugin_interface.h>

#include "Scene_cad_item.h"

#include "Scene_c3t3_cad_item.h"
#include "ui_CAD_refine_dialog.h"

#include "C3t3_cad_type.h"

#include <dtkCore>
#include <dtkContinuousGeometry>
#include <dtkContinuousGeometryUtils>
#include <dtkBRep>
#include <dtkBRepReader>
#include <dtkNurbsSurface>
#include <dtkClippedNurbsSurface>
#include <dtkClippedTrim>
#include <dtkRationalBezierCurve>
#include <dtkTrim>
#include <dtkTopoTrim>

#include <QObject>
#include <QAction>
#include <QMainWindow>
#include <QMenu>
#include <QApplication>
#include <QtPlugin>
#include <QInputDialog>
#include <QStringList>

// declare the CGAL function
CGAL::Three::Scene_item* cgal_code_cad_refine(QWidget* parent,
                                              dtkBRep*,
                                              const double sizing);

// To avoid verbose function and named parameters call
using namespace CGAL::parameters;

using namespace CGAL::Three;
class Polyhedron_demo_CAD_refine_plugin :
    public QObject,
    protected Polyhedron_demo_plugin_interface
{
  Q_OBJECT
  Q_INTERFACES(CGAL::Three::Polyhedron_demo_plugin_interface)
  Q_PLUGIN_METADATA(IID "com.geometryfactory.PolyhedronDemo.PluginInterface/1.0")

public:
  void init(QMainWindow* mainWindow, CGAL::Three::Scene_interface* scene_interface, Messages_interface*) {
    this->scene = scene_interface;
    this->mw = mainWindow;
    actionRefine = new QAction(tr("Refine"), mw);
    if(actionRefine != nullptr) {
        connect(actionRefine, SIGNAL(triggered()),
                this, SLOT(refine()));
    }
  }

  bool applicable(QAction*) const {
    return qobject_cast<Scene_c3t3_cad_item*>(scene->item(scene->selectionIndices().first()));
  }

  QList<QAction*> actions() const {
      return QList<QAction*>() << actionRefine;
  }
public Q_SLOTS:
    void refine();

private:
    QAction *actionRefine;
    Scene_interface *scene;
    QMainWindow *mw;
}; // end class Polyhedron_demo_remeshing_plugin

void Polyhedron_demo_CAD_refine_plugin::refine()
{
  Scene_c3t3_cad_item* c3t3_cad_item = qobject_cast<Scene_c3t3_cad_item*>(scene->item(scene->selectionIndices().first()));
  if(!c3t3_cad_item)
    return;
  C3t3& c3t3 = c3t3_cad_item->c3t3();
  const Mesh_domain& mesh_domain = c3t3_cad_item->meshDomain();
  std::cerr << "mesh domain adress in refine : " << &mesh_domain << std::endl;
  CGAL::Bbox_3 bbox = mesh_domain.bbox();
  std::cerr << bbox.xmin() << std::endl;
  QDialog dialog(mw);
  Ui::CADRefineDialog ui;
  ui.setupUi(&dialog);
  connect(ui.buttonBox, SIGNAL(accepted()),
          &dialog, SLOT(accept()));
  connect(ui.buttonBox, SIGNAL(rejected()),
          &dialog, SLOT(reject()));
  double diag = scene->len_diagonal();


  ui.sizeSpinBox->setDecimals(4);
  ui.sizeSpinBox->setRange(diag * 10e-6, // min
                           diag); // max
  ui.sizeSpinBox->setValue(diag * 0.05); // default value

  ui.errorSpinBox->setDecimals(6);
  ui.errorSpinBox->setRange(diag * 10e-7, // min
                            diag); // max
  ui.errorSpinBox->setValue(diag * 0.005);


  int status = dialog.exec();
  if(status == QDialog::Rejected)
    return;

  const double angle = ui.angleSpinBox->value();
  const double distance = ui.errorSpinBox->value();
  const double cell_sizing = ui.sizeSpinBox->value();

  Mesh_criteria p_criteria( cell_size = cell_sizing,
                            facet_distance = distance,
                            facet_angle = angle
                            );

  refine_mesh_3_impl(c3t3, mesh_domain, p_criteria, no_exude(), no_perturb(), no_odt(), no_lloyd(), false);
  c3t3_cad_item->c3t3_changed();
  c3t3_cad_item->resetCutPlane();
}

#include "CAD_refine_plugin.moc"
