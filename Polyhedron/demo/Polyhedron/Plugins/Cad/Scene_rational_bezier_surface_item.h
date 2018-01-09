#ifndef SCENE_RATIONAL_BEZIER_SURFACE_ITEM_H
#define SCENE_RATIONAL_BEZIER_SURFACE_ITEM_H


#ifdef scene_rational_bezier_surface_item_EXPORTS
#  define SCENE_RATIONAL_BEZIER_SURFACE_ITEM_EXPORT Q_DECL_EXPORT
#else
#  define SCENE_RATIONAL_BEZIER_SURFACE_ITEM_EXPORT Q_DECL_IMPORT
#endif

#include <CGAL/Three/Viewer_interface.h>
#include <CGAL/Three/Scene_item.h>

class dtkRationalBezierSurface;
struct Scene_rational_bezier_surface_item_priv;

class SCENE_RATIONAL_BEZIER_SURFACE_ITEM_EXPORT Scene_rational_bezier_surface_item
    :public CGAL::Three::Scene_item
{
   Q_OBJECT
public:
   Scene_rational_bezier_surface_item(const dtkRationalBezierSurface& dtk_rational_bezier_surface);
  ~Scene_rational_bezier_surface_item();
  bool isEmpty() const;
  void draw(CGAL::Three::Viewer_interface *) const;
  void drawEdges(CGAL::Three::Viewer_interface *) const;
  Scene_item* clone() const {return 0;}
  QString toolTip() const Q_DECL_OVERRIDE;
  bool supportsRenderingMode(RenderingMode m) const { return (m == Flat || m == FlatPlusEdges || m == Wireframe); }
  void compute_bbox() const;
  QMenu* contextMenu();

protected:
  friend struct Scene_rational_bezier_surface_item_priv;
  Scene_rational_bezier_surface_item_priv* d;
};

#endif // SCENE_RATIONAL_BEZIER_SURFACE_ITEM_H
