#ifndef SCENE_NURBS_ITEM_H
#define SCENE_NURBS_ITEM_H


#ifdef scene_nurbs_item_EXPORTS
#  define SCENE_NURBS_ITEM_EXPORT Q_DECL_EXPORT
#else
#  define SCENE_NURBS_ITEM_EXPORT Q_DECL_IMPORT
#endif

#include <CGAL/Three/Viewer_interface.h>
#include <CGAL/Three/Scene_item.h>
#include <CGAL/Three/Scene_group_item.h>

class dtkTopoTrim;
class dtkNurbsSurface;

struct Scene_nurbs_item_priv;

class SCENE_NURBS_ITEM_EXPORT Scene_nurbs_item
    :public CGAL::Three::Scene_group_item
{
   Q_OBJECT
public:
  Scene_nurbs_item(const dtkNurbsSurface& dtk_nurbs_surface, CGAL::Three::Scene_interface* scene);
  ~Scene_nurbs_item();
  bool isEmpty() const;

 public:
  void draw(CGAL::Three::Viewer_interface *) const;
  void drawEdges(CGAL::Three::Viewer_interface *) const;

 public:
  QString toolTip() const {return QString();}
  bool supportsRenderingMode(RenderingMode m) const { return (m == Flat || m == FlatPlusEdges || m == Wireframe); }

 public:
  Scene_item* clone() const {return 0;}
  Bbox bbox() const
  {
    if(!is_bbox_computed)
      compute_bbox();
    return _bbox;
  };
  void compute_bbox() const;
  QMenu* contextMenu();

public Q_SLOTS:
  void show_trimmed(bool b);
  void show_control_points(bool b);
  void highlight(const dtkTopoTrim *);

protected:
  friend struct Scene_nurbs_item_priv;
  Scene_nurbs_item_priv* d;
};

#endif // SCENE_NURBS_ITEM_H
