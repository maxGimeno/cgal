#include <CGAL/Three/Polyhedron_demo_io_plugin_interface.h>
#include <CGAL/Three/Three.h>
#include "Scene_spheres_item.h"
#include <fstream>
#include <string>
class Spheres_io_plugin :
    public QObject,
    public Polyhedron_demo_io_plugin_interface
{
  Q_OBJECT
  Q_INTERFACES(CGAL::Three::Polyhedron_demo_io_plugin_interface)
  Q_PLUGIN_METADATA(IID "com.geometryfactory.PolyhedronDemo.IOPluginInterface/1.0")

  QString name() const { return "Spheres_io_plugin"; }

  QString nameFilters() const { return "Spheres_item files (*.spheres.txt)"; }

  bool canLoad(QFileInfo) const { return true; }

  QList<CGAL::Three::Scene_item*> load(QFileInfo fileinfo, bool& ok, bool add_to_scene=true)
  {
    // Open file
    std::ifstream in(fileinfo.filePath().toUtf8());
    if(!in) {
      std::cerr << "Error! Cannot open file " << (const char*)fileinfo.filePath().toUtf8() << std::endl;
      ok = false;
      return QList<CGAL::Three::Scene_item*>();
    }
    std::size_t max_index(0), nb_spheres(0);
    std::string line;
    std::getline(in, line);
    std::istringstream header_line(line);
    //get max index and number of spheres
    header_line >> nb_spheres;
    header_line >> max_index;
    Scene_spheres_item* item = new Scene_spheres_item(0, max_index, false);
    item->setColor(QColor(120,120,120));
    //get spheres and fill item
    while(nb_spheres--)
    {
      if(!std::getline(in, line))
      {
        std::cerr<<"Wrong number of spheres in file."<<std::endl;
        delete item;
        ok = false;
        return QList<CGAL::Three::Scene_item*>();
      }
      std::istringstream sphere_line(line);
      std::size_t id;
      float x(0),y(0),z(0),r(0);
      sphere_line >> id;
      sphere_line >> x;
      sphere_line >> y;
      sphere_line >> z;
      sphere_line >> r;
      item->add_sphere(
            Scene_spheres_item::Sphere(
              Scene_spheres_item::Kernel::Point_3(x,y,z), r*r),
            id);
    }
    item->setName(fileinfo.baseName());
    item->setRenderingMode(Gouraud);
    if(add_to_scene)
    {
      CGAL::Three::Three::scene()->addItem(item);
    }
    ok = true;
    return QList<CGAL::Three::Scene_item*>()<< item;
  }


  bool canSave(const CGAL::Three::Scene_item* scene_item) {
      return qobject_cast<const Scene_spheres_item*>(scene_item);
  }

  bool save(QFileInfo fileinfo,QList<CGAL::Three::Scene_item*>& items)
  {
    Scene_item* scene_item = items.front();
    items.pop_front();
    const Scene_spheres_item* spheres_item = qobject_cast<const Scene_spheres_item*>(scene_item);
    if(!spheres_item)
      return false;
    return spheres_item->save(fileinfo.filePath().toStdString());
  }

};


#include "Spheres_io_plugin.moc"
