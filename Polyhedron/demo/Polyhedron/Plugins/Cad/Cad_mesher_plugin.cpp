#include "config.h"
#include <CGAL/Three/Polyhedron_demo_plugin_interface.h>
#include "Scene_cad_item.h"
#include "Scene_c3t3_item.h"
#include "ui_Cad_mesher_dialog.h"

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Mesh_triangulation_3.h>
#include <CGAL/Mesh_complex_3_in_triangulation_3.h>
#include <CGAL/Mesh_criteria_3.h>
#include <CGAL/cgalMeshDomainWithRationalBezierFeatures.h>

#include <CGAL/make_mesh_3.h>
#include <CGAL/perturb_mesh_3.h>
#include <CGAL/exude_mesh_3.h>

#include <CGAL/IO/File_binary_mesh_3.h>

#include <cgalBrepMeshDomainData.h>

#include <dtkCore>
#include <dtkContinuousGeometry>
#include <dtkContinuousGeometryUtils>
#include <dtkBRep>
#include <dtkBRepReader>
#include <dtkNurbsSurface>

#include <QObject>
#include <QAction>
#include <QMainWindow>
#include <QMenu>
#include <QApplication>
#include <QtPlugin>
#include <QInputDialog>
#include <QStringList>

// declare the CGAL function
CGAL::Three::Scene_item* cgal_code_cad_remesh(QWidget* parent,
                                              dtkBRep*,
                                              const double angle,
                                              const double sizing,
                                              const double approx,
                                              int tag);

// /////////////////////////////////////////////////////////////////
// Domains
// /////////////////////////////////////////////////////////////////
typedef CGAL::cgalMeshDomainWithRationalBezierFeatures< cgalBrepMeshDomainData< Kernel > > Mesh_domain_with_features;

// /////////////////////////////////////////////////////////////////
//  Triangulation
// /////////////////////////////////////////////////////////////////
typedef CGAL::Mesh_triangulation_3< Mesh_domain_with_features >::type Tr;
typedef CGAL::Mesh_complex_3_in_triangulation_3<Tr> C3t3;

// // /////////////////////////////////////////////////////////////////
// // Criteria
// // /////////////////////////////////////////////////////////////////
typedef CGAL::Mesh_criteria_3<Tr> Mesh_criteria;
typedef Mesh_criteria::Facet_criteria Facet_criteria;
typedef Mesh_criteria::Cell_criteria Cell_criteria;

using namespace CGAL::Three;
class Polyhedron_demo_remeshing_plugin :
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
    actionRemeshing = new QAction(tr("CadRemeshing"), mw);
    actionRemeshing->setProperty("subMenuName", "Volume Mesh Generation");
    if(actionRemeshing) {
      connect(actionRemeshing, SIGNAL(triggered()),
              this, SLOT(remesh()));
    }
  }

  bool applicable(QAction*) const {
    return qobject_cast<Scene_cad_item*>(scene->item(scene->selectionIndices().first()));
  }

  QList<QAction*> actions() const {
    return QList<QAction*>() << actionRemeshing;
  }
public Q_SLOTS:
  void remesh();

private:
  QAction* actionRemeshing;
  Scene_interface *scene;
  QMainWindow *mw;
}; // end class Polyhedron_demo_remeshing_plugin

void Polyhedron_demo_remeshing_plugin::remesh()
{
  Scene_cad_item* cad_item = qobject_cast<Scene_cad_item*>(scene->item(scene->selectionIndices().first()));
  if(!cad_item)
    return;

  dtkBRep* brep = cad_item->brep();
  if(!brep) return;

  QDialog dialog(mw);
  Ui::CadMesherDialog ui;
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


  int i = dialog.exec();
  if(i == QDialog::Rejected)
    return;

  const double angle = ui.angleSpinBox->value();
  const double distance = ui.errorSpinBox->value();
  const double cell_sizing = ui.sizeSpinBox->value();

  Mesh_criteria p_criteria( cell_size = cell_sizing,
                            facet_distance = distance,
                            facet_angle = angle);

  Mesh_domain_with_features cgal_brep_mesh_domain_with_features(*brep);
  // 	Mesh generation (without optimization)
  C3t3 p_c3t3 = CGAL::make_mesh_3<C3t3>(cgal_brep_mesh_domain_with_features,
                                        p_criteria, no_perturb(), no_exude());
  if(!p_c3t3.is_valid()){std::cerr << "bip biip not valid" << std::endl;}

  std::cerr << "number of cells : "<< p_c3t3.number_of_cells() << std::endl;
  std::cerr << "number of cells in complex : "<< p_c3t3.number_of_cells_in_complex() << std::endl;

  // Output
  Scene_c3t3_item* c3t3_item = new Scene_c3t3_item(p_c3t3);
  if(!c3t3_item)
  {
    qDebug()<<"c3t3 item not created";
    return;
  }
  c3t3_item->setName(QString("%1 (c3t3)").arg(cad_item->name()));
  scene->addItem(c3t3_item);
}

#include "Cad_mesher_plugin.moc"
