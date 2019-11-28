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
  Scene_nurbs_item(const dtkNurbsSurface& dtk_nurbs_surface);
  ~Scene_nurbs_item();
  bool isEmpty() const Q_DECL_OVERRIDE;
  void draw(CGAL::Three::Viewer_interface *) const Q_DECL_OVERRIDE;
  void drawEdges(CGAL::Three::Viewer_interface *) const Q_DECL_OVERRIDE;
  Scene_item* clone() const Q_DECL_OVERRIDE {return 0;}
  QString toolTip() const Q_DECL_OVERRIDE;
    void compute_bbox() const Q_DECL_OVERRIDE;
  QMenu* contextMenu() Q_DECL_OVERRIDE;
  bool supportsRenderingMode(RenderingMode m) const Q_DECL_OVERRIDE { return (m == Gouraud || m == GouraudPlusEdges || m == Wireframe); }
  void initializeBuffers(Viewer_interface *) const;

  Bbox bbox() const
  {
    if(!is_bbox_computed)
      compute_bbox();
    return _bbox;
  };
public Q_SLOTS:
  void show_trimmed(bool b);
  void show_control_points(bool b);
  void show_bezier_surfaces(bool b);
  void highlight(const dtkTopoTrim *);

protected:
  friend struct Scene_nurbs_item_priv;
  Scene_nurbs_item_priv* d;
};

#endif // SCENE_NURBS_ITEM_H
