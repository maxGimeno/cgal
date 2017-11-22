#include <CGAL/Three/Polyhedron_demo_io_plugin_interface.h>
#include <CGAL/Three/Polyhedron_demo_plugin_interface.h>

//because dtk uses qt's foreach keyword, which we do not allow
#define foreach Q_FOREACH

#include <dtkCore>
#include <dtkContinuousGeometry>
#include <dtkContinuousGeometryUtils>
#include <dtkBRep>
#include <dtkBRepReader>
#include <dtkNurbsSurface>

#include <QFileInfo>
#include <QDebug>

#include "Scene_cad_item.h"
#include "Messages_interface.h"
#include "Viewer.h"
#include "Scene.h"
#include "MainWindow.h"

class Cad_meshing_plugin :
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
  QString name() const { return "Cad_meshing_plugin"; }
  QString nameFilters() const { return "Cad Files (*.3dm)"; }
  bool canLoad() const { return true; }
  CGAL::Three::Scene_item* load(QFileInfo fileinfo) {
      if(fileinfo.suffix().toLower() != "3dm") return 0;

      dtkContinuousGeometrySettings settings;
      settings.beginGroup("continuous-geometry");

      dtkContinuousGeometry::setVerboseLoading(true);
      dtkContinuousGeometry::initialize(settings.value("plugins").toString());
      settings.endGroup();
      dtkBRepReader* brep_reader = dtkContinuousGeometry::bRepReader::pluginFactory().create("openNURBSBRepReader");
      if(brep_reader == NULL) {
        messageInterface->error("ERROR in initialization");
        return NULL;
      }
      brep_reader->setInputBRepFilePath(fileinfo.absoluteFilePath());
      brep_reader->run();
      dtkBRep* brep = brep_reader->outputDtkBRep();
      Scene_cad_item* item = new Scene_cad_item(brep, scene);
      item->setName(fileinfo.baseName());
      item->setFlatMode();
      item->show_intersections(true);
      QList<int> children_ids;
      Q_FOREACH(CGAL::Three::Scene_item* child, item->getChildren())
      {
        children_ids.append(scene->item_id(child));
      }
      static_cast<Scene*>(scene)->setSelectedItemsList(children_ids);
      static_cast<MainWindow*>(this->mw)->colorItems();
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
#include "Cad_meshing_plugin.moc"
