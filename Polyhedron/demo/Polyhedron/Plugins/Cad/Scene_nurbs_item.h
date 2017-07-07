#ifndef SCENE_NURBS_ITEM_H
#define SCENE_NURBS_ITEM_H


#ifdef scene_nurbs_item_EXPORTS
#  define SCENE_NURBS_ITEM_EXPORT Q_DECL_EXPORT
#else
#  define SCENE_NURBS_ITEM_EXPORT Q_DECL_IMPORT
#endif


#include <CGAL/Three/Viewer_interface.h>
#include <CGAL/Three/Scene_item.h>
class dtkNurbsSurface;
struct Scene_nurbs_item_priv;
class SCENE_NURBS_ITEM_EXPORT Scene_nurbs_item
    :public CGAL::Three::Scene_item
{
   Q_OBJECT
public:
  Scene_nurbs_item(const dtkNurbsSurface& dtk_nurbs_surface);
  ~Scene_nurbs_item();
  bool isEmpty() const;
  void draw(CGAL::Three::Viewer_interface *) const;
  Scene_item* clone() const {return 0;}
  QString toolTip() const {return QString();}
  bool supportsRenderingMode(RenderingMode m) const { return (m == Flat); }
  void compute_bbox() const;
protected:
  friend struct Scene_nurbs_item_priv;
  Scene_nurbs_item_priv* d;
};

#endif // SCENE_NURBS_ITEM_H
