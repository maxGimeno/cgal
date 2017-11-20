#include <CGAL/Three/Polyhedron_demo_io_plugin_interface.h>
#include <CGAL/Three/Polyhedron_demo_plugin_interface.h>

//because dtk uses qt's foreach keyword, which we do not allow
#define foreach Q_FOREACH

#include <dtkCore>
#include <dtkContinuousGeometry>
#include <dtkContinuousGeometryUtils>

#include <dtkRationalBezierSurface>

#include <QFileInfo>
#include <QDebug>

#include "Scene_rational_bezier_surface_item.h"
#include "Messages_interface.h"
#include "Viewer.h"
#include "Scene.h"
#include "MainWindow.h"

class Rational_bezier_surface_plugin :
    public QObject,
    public CGAL::Three::Polyhedron_demo_plugin_interface,
    public CGAL::Three::Polyhedron_demo_io_plugin_interface
{
  Q_OBJECT
  Q_INTERFACES(CGAL::Three::Polyhedron_demo_io_plugin_interface)
  Q_PLUGIN_METADATA(IID "com.geometryfactory.PolyhedronDemo.IOPluginInterface/1.0")
  Q_INTERFACES(CGAL::Three::Polyhedron_demo_plugin_interface)
  Q_INTERFACES(CGAL::Three::Polyhedron_demo_io_plugin_interface)
  Q_PLUGIN_METADATA(IID "com.geometryfactory.PolyhedronDemo.PluginInterface/1.0")
public:
  //PLUGIN PART (for access to the scene and the message interface)
  bool applicable(QAction*) const
   {
     return false;
   }
   QList<QAction*> actions() const
   {
     return QList<QAction*>();
   }
   void init(QMainWindow* mainWindow, CGAL::Three::Scene_interface* sc, Messages_interface* mi) Q_DECL_OVERRIDE
   {
     this->messageInterface = mi;
     this->scene = sc;
     this->mw = mainWindow;
   }


  //IO PLUGIN PART
  QString name() const { return "Rational_bezier_surface_plugin"; }
  QString nameFilters() const { return "None"; }
  bool canLoad() const { return true; }
  CGAL::Three::Scene_item* load(QFileInfo fileinfo) {
      dtkContinuousGeometrySettings settings;
      settings.beginGroup("continuous-geometry");
      dtkLogger::instance().attachConsole();
      dtkLogger::instance().setLevel(dtkLog::Error);
      dtkContinuousGeometry::setVerboseLoading(true);
      dtkContinuousGeometry::initialize(settings.value("plugins").toString());
      settings.endGroup();
      dtkAbstractRationalBezierSurfaceData *bezier_surface_data = dtkContinuousGeometry::abstractRationalBezierSurfaceData::pluginFactory().create("dtkRationalBezierSurfaceDataOn");
      if(bezier_surface_data == nullptr) {
          dtkFatal() << "The openNURBS plugin of dtkAbstractRationalBezierSurfaceData could not be loaded.";
      }
      bezier_surface_data->create(fileinfo.absoluteFilePath().toStdString());
      dtkRationalBezierSurface *bezier_surface = new dtkRationalBezierSurface(bezier_surface_data);

      Scene_rational_bezier_surface_item* item = new Scene_rational_bezier_surface_item(*bezier_surface);
      item->setName(fileinfo.baseName());
      item->setFlatMode();
      scene->setSelectedItem(scene->item_id(item));
      return item;
  }

  bool canSave(const CGAL::Three::Scene_item*) {
      return false;
  }

  bool save(const CGAL::Three::Scene_item*, QFileInfo) {
    return false;
  }


  Messages_interface* messageInterface;
  //The reference to the scene
  CGAL::Three::Scene_interface* scene;
  //The reference to the main window
  QMainWindow* mw;
};
#include "Rational_bezier_surface_plugin.moc"
