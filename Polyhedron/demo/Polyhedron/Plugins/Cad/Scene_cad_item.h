#ifndef SCENE_CAD_ITEM_H
#define SCENE_CAD_ITEM_H

#ifdef scene_cad_item_EXPORTS
#  define SCENE_CAD_ITEM_EXPORT Q_DECL_EXPORT
#else
#  define SCENE_CAD_ITEM_EXPORT Q_DECL_IMPORT
#endif

#include <CGAL/Three/Scene_item.h>
#include <CGAL/Three/Viewer_interface.h>
#include <QFileInfo>
#include <CGAL/Three/Scene_group_item.h>

class dtkBRep;
struct Scene_cad_item_priv;
class SCENE_CAD_ITEM_EXPORT Scene_cad_item :  public CGAL::Three::Scene_group_item
{
  Q_OBJECT
public :
  Scene_cad_item(dtkBRep*, CGAL::Three::Scene_interface* scene);
  ~Scene_cad_item();

  bool supportsRenderingMode(RenderingMode m) const Q_DECL_OVERRIDE {
    return (m == Flat || m == FlatPlusEdges || m == Wireframe);
  }

  void draw(CGAL::Three::Viewer_interface* viewer) const Q_DECL_OVERRIDE;
  void drawEdges(Viewer_interface *) const Q_DECL_OVERRIDE;
  void invalidateOpenGLBuffers() Q_DECL_OVERRIDE{}

  void computeElements() const;
  Scene_item* clone() const Q_DECL_OVERRIDE {return 0;}
  QString toolTip() const Q_DECL_OVERRIDE;
  Bbox bbox()const Q_DECL_OVERRIDE;
  QMenu* contextMenu();
  dtkBRep* brep();
  void checkTrimToProtect(int i);
  void clearHighlight();
  const std::vector<int>& trimsToProtect()const;
  const dtkBRep* brep()const;
public Q_SLOTS:
  void show_trimmed(bool b);
  void show_intersections(bool b);
protected:
  friend struct Scene_cad_item_priv;
  Scene_cad_item_priv* d;

}; //end of class Scene_cad_item

#endif // SCENE_CAD_ITE_H
