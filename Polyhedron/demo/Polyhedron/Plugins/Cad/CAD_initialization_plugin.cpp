#include <CGAL/Three/Polyhedron_demo_plugin_interface.h>

#include "Scene_cad_item.h"

#include "Scene_c3t3_cad_item.h"
#include "ui_CAD_mesher_protect_initialization_dialog.h"

#include "C3t3_cad_type.h"

#include <dtkCore>
#include <dtkContinuousGeometry>
#include <dtkContinuousGeometryUtils>
#include <dtkBRep>
#include <dtkBRepReader>
#include <dtkNurbsSurface>
#include <dtkClippedNurbsSurface>
#include <dtkClippedTrim>
#include <dtkClippedTrimLoop>
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
CGAL::Three::Scene_item* cgal_code_cad_protect_intialization(QWidget* parent,
                                              dtkBRep*,
                                              const double sizing);

// To avoid verbose function and named parameters call
using namespace CGAL::parameters;

using namespace CGAL::Three;
class Polyhedron_demo_CAD_initialization_plugin :
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
    actionProtectInitialization = new QAction(tr("Initialization by protection"), mw);
    actionRandomInitialization = new QAction(tr("Initialization by random shooting"), mw);
    actionSamplingInitialization = new QAction(tr("Initialization by sampling the surfaces"), mw);
    actionProtectInitialization->setProperty("subMenuName", "Mesh initialization");
    actionRandomInitialization->setProperty("subMenuName", "Mesh initialization");
    actionSamplingInitialization->setProperty("subMenuName", "Mesh initialization");
    if(actionProtectInitialization) {
      connect(actionProtectInitialization, SIGNAL(triggered()),
              this, SLOT(protectInitialization()));
    }
    if(actionRandomInitialization != nullptr) {
      connect(actionRandomInitialization, SIGNAL(triggered()),
              this, SLOT(randomInitialization()));
    }
    if(actionSamplingInitialization != nullptr) {
        connect(actionSamplingInitialization, SIGNAL(triggered()),
                this, SLOT(samplingInitialization()));
    }
  }

  bool applicable(QAction*) const {
    return qobject_cast<Scene_cad_item*>(scene->item(scene->selectionIndices().first()));
  }

  QList<QAction*> actions() const {
      return QList<QAction*>() << actionProtectInitialization << actionRandomInitialization << actionSamplingInitialization;
  }
public Q_SLOTS:
    void protectInitialization();
    void randomInitialization();
    void samplingInitialization();

private:
    QAction *actionProtectInitialization;
    QAction *actionRandomInitialization;
    QAction *actionSamplingInitialization;
    Scene_interface *scene;
    QMainWindow *mw;
}; // end class Polyhedron_demo_remeshing_plugin

void Polyhedron_demo_CAD_initialization_plugin::protectInitialization()
{
    std::cerr << "Passes : " << Q_FUNC_INFO << std::endl;
  Scene_cad_item* cad_item = qobject_cast<Scene_cad_item*>(scene->item(scene->selectionIndices().first()));
  if(!cad_item)
    return;

  dtkBRep* brep = cad_item->brep();
  if(!brep) return;

  QDialog dialog(mw);
  Ui::CADMesherProtectInitializationDialog ui;
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

  int i = dialog.exec();
  if(i == QDialog::Rejected)
    return;

  const double edge_sizing = ui.sizeSpinBox->value();
  Mesh_domain_with_features* cgal_brep_mesh_domain_with_features = new Mesh_domain_with_features(*brep);
  ///////////////////////////////////////////////////////////////////
  //    Recovers the features
  ///////////////////////////////////////////////////////////////////
  const std::vector< dtkNurbsSurface *>& surfs = brep->nurbsSurfaces();
  std::vector< dtkClippedNurbsSurface* > c_surfs;
  for (auto surf : surfs) {
      c_surfs.push_back(new dtkClippedNurbsSurface(*(surf)));
  }

  std::list< std::tuple< dtkNurbsCurve *, dtkNurbsCurve *, dtkNurbsCurve * > > features;
  std::map< const dtkTopoTrim *, dtkNurbsCurve * > topo_trims;

  // ///////////////////////////////////////////////////////////////////
  // Iterates on all the trims, check if the topo_trim has been found, if it has, add the three curves as a tuple
  // Else add the topo trim and the first trim found attached to it
  // As the BRep model is a closed polysurface, for each topo trim there should be two trims associated to it
  // ///////////////////////////////////////////////////////////////////
  for (auto c_surf : c_surfs) {
      for (auto c_trim_loop : c_surf->m_clipped_trim_loops) {
          for (auto c_trim : c_trim_loop->m_clipped_trims) {
              if(c_trim->m_trim.topoTrim()->m_nurbs_curve_3d != nullptr) {
                  auto topo_trim = topo_trims.find(c_trim->m_trim.topoTrim());
                  if (topo_trim == topo_trims.end()) {
                      topo_trims.insert(std::make_pair(c_trim->m_trim.topoTrim(), c_trim->m_nurbs_curve));
                  } else {
                      features.push_back(std::make_tuple(topo_trim->second, topo_trim->first->m_nurbs_curve_3d, c_trim->m_nurbs_curve));
                  }
              }
          }
      }
  }

  cgal_brep_mesh_domain_with_features->add_features(features.begin(), features.end());
  std::cerr << "mesh domain adress in intialize : " << cgal_brep_mesh_domain_with_features << std::endl;
  Mesh_criteria p_criteria( edge_size = edge_sizing);

  C3t3 c3t3;
  CGAL::internal::Mesh_3::init_c3t3_with_features(c3t3, *cgal_brep_mesh_domain_with_features, p_criteria, false);

  // //Output
  Scene_c3t3_cad_item *c3t3_cad_item = new Scene_c3t3_cad_item(c3t3, *cgal_brep_mesh_domain_with_features);
  if(!c3t3_cad_item)
  {
    qDebug()<<"c3t3 CAD item not created";
    return;
  }
  c3t3_cad_item->setName(QString("%1 (c3t3)").arg(cad_item->name()));
  scene->addItem(c3t3_cad_item);
}

void Polyhedron_demo_CAD_initialization_plugin::randomInitialization(){

}

void Polyhedron_demo_CAD_initialization_plugin::samplingInitialization(){

}

#include "CAD_initialization_plugin.moc"
