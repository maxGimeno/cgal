#include <CGAL/Three/Polyhedron_demo_plugin_interface.h>

#include "Scene_cad_item.h"

#include "Scene_c3t3_cad_item.h"
#include "ui_CAD_mesher_protect_initialization_dialog.h"

#include "C3t3_cad_type.h"

//dtk
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

//dtk-nurbs-probing
#include <dtkSeamProtectionGraph>

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
  Polyhedron_demo_CAD_initialization_plugin() : ui_protection(nullptr) {};
  virtual ~Polyhedron_demo_CAD_initialization_plugin();

  void init(QMainWindow* mainWindow, CGAL::Three::Scene_interface* scene_interface, Messages_interface*) {
    this->scene = scene_interface;
    this->mw = mainWindow;
    actionProtectInitialization = new QAction(tr("Initialization by protection"), mw);
    actionRandomShootingInitialization = new QAction(tr("Initialization by random shooting"), mw);
    actionSamplingInitialization = new QAction(tr("Initialization by sampling the surfaces"), mw);
    actionProtectInitialization->setProperty("subMenuName", "Mesh initialization");
    actionRandomShootingInitialization->setProperty("subMenuName", "Mesh initialization");
    actionSamplingInitialization->setProperty("subMenuName", "Mesh initialization");
    if(actionProtectInitialization) {
      connect(actionProtectInitialization, SIGNAL(triggered()),
              this, SLOT(protectInitialization()));
    }
    if(actionRandomShootingInitialization != nullptr) {
      connect(actionRandomShootingInitialization, SIGNAL(triggered()),
              this, SLOT(randomShootingInitialization()));
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
      return QList<QAction*>() << actionProtectInitialization << actionRandomShootingInitialization << actionSamplingInitialization;
  }

public Q_SLOTS:
    void protectInitialization();
    void randomShootingInitialization();
    void samplingInitialization();

    void updateProtectTrim(int);

private:
    QAction *actionProtectInitialization;
    QAction *actionRandomShootingInitialization;
    QAction *actionSamplingInitialization;
    Scene_interface *scene;
    QMainWindow *mw;

    Ui::CADMesherProtectInitializationDialog *ui_protection;
}; // end class Polyhedron_demo_remeshing_plugin

Polyhedron_demo_CAD_initialization_plugin::~Polyhedron_demo_CAD_initialization_plugin()
{
    if(ui_protection != nullptr) {
        delete ui_protection;
        ui_protection = nullptr;
    }
}

void Polyhedron_demo_CAD_initialization_plugin::updateProtectTrim(int cs)
{
    if(cs == 2) {
        ui_protection->mergingToleranceSB->setDisabled(false);
    } else if(cs == 1) {
        ui_protection->mergingToleranceSB->setDisabled(false);
    } else {
        ui_protection->mergingToleranceSB->setDisabled(true);
    }
}



void Polyhedron_demo_CAD_initialization_plugin::protectInitialization()
{
    Scene_cad_item* cad_item = qobject_cast<Scene_cad_item*>(scene->item(scene->selectionIndices().first()));
    if(!cad_item) {return;}

    dtkBRep* brep = cad_item->brep();
    if(!brep) return;

    QDialog dialog(mw);
    ui_protection = new Ui::CADMesherProtectInitializationDialog();
    ui_protection->setupUi(&dialog);
    ui_protection->trimsGroupBox->hide();
    connect(ui_protection->protectTrimCB, &QCheckBox::stateChanged,
            [this, cad_item](){
      ui_protection->trimsGroupBox->setVisible(ui_protection->protectTrimCB->isChecked());
    });
    connect(ui_protection->buttonBox, SIGNAL(accepted()),
            &dialog, SLOT(accept()));
    connect(ui_protection->buttonBox, SIGNAL(rejected()),
            &dialog, SLOT(reject()));
    double diag = scene->len_diagonal();

    connect(ui_protection->protectTrimCB, SIGNAL(stateChanged(int)), this, SLOT(updateProtectTrim(int)));
    ui_protection->mergingToleranceSB->setDisabled(true);

    ui_protection->sizeSpinBox->setDecimals(4);
    ui_protection->sizeSpinBox->setRange(diag * 10e-6, // min
                                         diag); // max
    ui_protection->sizeSpinBox->setValue(diag * 0.05); // default value

    std::unordered_map< const dtkTopoTrim *, std::size_t > tts_map;
    std::vector<bool> is_protected(brep->topoTrims().size());
    for(std::size_t i=0; i<is_protected.size(); ++i)
      is_protected[i] = false;
    for(std::size_t i=0; i< brep->topoTrims().size(); ++i)
    {
      tts_map.insert(std::make_pair(brep->topoTrims()[i], i));
      QCheckBox* checkbox = new QCheckBox(QString("trim %1").arg(i), &dialog);
      connect(checkbox, &QCheckBox::stateChanged,
              [checkbox, cad_item, i, &is_protected](){
        cad_item->checkTrimToProtect(static_cast<int>(i));
        if(checkbox->checkState() == Qt::Checked)
          is_protected[i] = true;

      });
      checkbox->setChecked(true);
      ui_protection->trimsGroupBox->layout()->addWidget(checkbox);
    }

    int i = dialog.exec();
    if(i == QDialog::Rejected)
        return;

    const double edge_sizing = ui_protection->sizeSpinBox->value();
    Mesh_domain_with_features* cgal_brep_mesh_domain_with_features = new Mesh_domain_with_features(*brep);
    ///////////////////////////////////////////////////////////////////
    //    Recovers the features
    ///////////////////////////////////////////////////////////////////
    dtkSeamProtectionGraph *protection_graph = nullptr;
    if(ui_protection->protectTrimCB->checkState() == Qt::Checked) {
        protection_graph = new dtkSeamProtectionGraph(*brep, ui_protection->sizeSpinBox->value(), ui_protection->mergingToleranceSB->value(), false);
    } else {
        protection_graph = new dtkSeamProtectionGraph(*brep, ui_protection->sizeSpinBox->value(), 0., true);
    }

    C3t3 p_c3t3;
    Tr& tr = p_c3t3.triangulation();
    const CGAL::Bbox_3& cgal_bbox = cgal_brep_mesh_domain_with_features->bbox();
    std::list< Weighted_point > w_points;
    w_points.push_back(Weighted_point(Point_3(cgal_bbox.xmax(), cgal_bbox.ymin(), cgal_bbox.zmin()), 1.));
    w_points.push_back(Weighted_point(Point_3(cgal_bbox.xmin(), cgal_bbox.ymin(), cgal_bbox.zmin()), 1.));
    w_points.push_back(Weighted_point(Point_3(cgal_bbox.xmin(), cgal_bbox.ymin(), cgal_bbox.zmin()), 1.));
    w_points.push_back(Weighted_point(Point_3(cgal_bbox.xmin(), cgal_bbox.ymax(), cgal_bbox.zmin()), 1.));
    w_points.push_back(Weighted_point(Point_3(cgal_bbox.xmax(), cgal_bbox.ymax(), cgal_bbox.zmin()), 1.));
    w_points.push_back(Weighted_point(Point_3(cgal_bbox.xmin(), cgal_bbox.ymin(), cgal_bbox.zmax()), 1.));
    w_points.push_back(Weighted_point(Point_3(cgal_bbox.xmax(), cgal_bbox.ymin(), cgal_bbox.zmax()), 1.));
    w_points.push_back(Weighted_point(Point_3(cgal_bbox.xmin(), cgal_bbox.ymax(), cgal_bbox.zmax()), 1.));
    w_points.push_back(Weighted_point(Point_3(cgal_bbox.xmax(), cgal_bbox.ymax(), cgal_bbox.zmax()), 1.));
    for(auto& w_point : w_points) {
        Vertex_handle vi = tr.insert(w_point);
        CGAL_assertion(vi != Vertex_handle());
        p_c3t3.set_dimension(vi, 0);
        p_c3t3.set_index(vi, 0);
    }

    std::size_t index = 1;
    std::size_t curr_index = 0;
    for(auto& p_sphere : protection_graph->m_protection_spheres) {
        auto find_tt = protection_graph->m_map.find(p_sphere->m_bezier_curve);

        if(is_protected[tts_map[find_tt->second]])
          continue;
        // if(find_tt == protection_graph->m_map.end()) { dtkFatal() << "Mistmatching between bezier curve pointers on spheres and in map";}
        // auto find = tts_map.find(find_tt->second);
        // if( find == tts_map.end()) {
        //     curr_index = index;
        //     tts_map.insert(std::make_pair(find_tt->second, index));
        //     ++index;
        // } else {
        //     curr_index = find->second;
        // }
        Weighted_point pi(Point_3(p_sphere->center()[0], p_sphere->center()[1], p_sphere->center()[2]),
                          p_sphere->radius() * p_sphere->radius());
        Vertex_handle v = tr.insert(pi);
        // `v` could be null if `pi` is hidden by other vertices of `tr`.
        CGAL_assertion(v != Vertex_handle());
        if(v == Vertex_handle()) {
            dtkInfo() << "A vertex is hidden by its neighbors and will be removed";
        }
        p_c3t3.set_dimension(v, 1); // by construction, points are on surface
        p_c3t3.set_index(v, 0// curr_index
                         );//TODO replace by curve index
    }

    // //Output
    Scene_c3t3_cad_item *c3t3_cad_item = new Scene_c3t3_cad_item(p_c3t3, *cgal_brep_mesh_domain_with_features);
    if(!c3t3_cad_item)
        {
            qDebug()<<"c3t3 CAD item not created";
            return;
        }
    c3t3_cad_item->setName(QString("%1 (c3t3)").arg(cad_item->name()));
    scene->addItem(c3t3_cad_item);
}


void Polyhedron_demo_CAD_initialization_plugin::randomShootingInitialization(){

    Scene_cad_item* cad_item = qobject_cast<Scene_cad_item*>(scene->item(scene->selectionIndices().first()));
    if(!cad_item) {return;}

    dtkBRep* brep = cad_item->brep();
    if(!brep) return;

    Mesh_domain_with_features* cgal_brep_mesh_domain_with_features = new Mesh_domain_with_features(*brep);
    C3t3 p_c3t3;
    Tr& tr = p_c3t3.triangulation();
    const CGAL::Bbox_3& cgal_bbox = cgal_brep_mesh_domain_with_features->bbox();
    std::list< Weighted_point > w_points;
    w_points.push_back(Weighted_point(Point_3(cgal_bbox.xmax(), cgal_bbox.ymin(), cgal_bbox.zmin()), 1.));
    w_points.push_back(Weighted_point(Point_3(cgal_bbox.xmin(), cgal_bbox.ymin(), cgal_bbox.zmin()), 1.));
    w_points.push_back(Weighted_point(Point_3(cgal_bbox.xmin(), cgal_bbox.ymin(), cgal_bbox.zmin()), 1.));
    w_points.push_back(Weighted_point(Point_3(cgal_bbox.xmin(), cgal_bbox.ymax(), cgal_bbox.zmin()), 1.));
    w_points.push_back(Weighted_point(Point_3(cgal_bbox.xmax(), cgal_bbox.ymax(), cgal_bbox.zmin()), 1.));
    w_points.push_back(Weighted_point(Point_3(cgal_bbox.xmin(), cgal_bbox.ymin(), cgal_bbox.zmax()), 1.));
    w_points.push_back(Weighted_point(Point_3(cgal_bbox.xmax(), cgal_bbox.ymin(), cgal_bbox.zmax()), 1.));
    w_points.push_back(Weighted_point(Point_3(cgal_bbox.xmin(), cgal_bbox.ymax(), cgal_bbox.zmax()), 1.));
    w_points.push_back(Weighted_point(Point_3(cgal_bbox.xmax(), cgal_bbox.ymax(), cgal_bbox.zmax()), 1.));
    for(auto& w_point : w_points) {
        Vertex_handle vi = tr.insert(w_point);
        CGAL_assertion(vi != Vertex_handle());
        p_c3t3.set_dimension(vi, 0);
        p_c3t3.set_index(vi, 0);
    }

    ///////////////////////////////////////////////////////////////////
    //    Recovers the points
    ///////////////////////////////////////////////////////////////////
    std::vector<std::pair<Point_3, Index> > initial_points;
    cgal_brep_mesh_domain_with_features->construct_initial_points_object()(std::back_inserter(initial_points), 20);

    // cgal_brep_mesh_domain_with_features.
    for(auto& point : initial_points) {
        Vertex_handle vi = tr.insert(Weighted_point(point.first, 1.));
        CGAL_assertion(vi != Vertex_handle());
        p_c3t3.set_dimension(vi, 2);
        p_c3t3.set_index(vi, point.second);
    }

    // //Output
    Scene_c3t3_cad_item *c3t3_cad_item = new Scene_c3t3_cad_item(p_c3t3, *cgal_brep_mesh_domain_with_features);
    if(!c3t3_cad_item)
        {
            qDebug()<<"c3t3 CAD item not created";
            return;
        }
    c3t3_cad_item->setName(QString("%1 (c3t3)").arg(cad_item->name()));
    scene->addItem(c3t3_cad_item);
}

void Polyhedron_demo_CAD_initialization_plugin::samplingInitialization(){

}

#include "CAD_initialization_plugin.moc"
