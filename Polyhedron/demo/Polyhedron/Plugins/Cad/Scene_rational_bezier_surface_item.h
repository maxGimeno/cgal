#ifndef SCENE_RATIONAL_BEZIER_SURFACE_ITEM_H
#define SCENE_RATIONAL_BEZIER_SURFACE_ITEM_H


#ifdef scene_rational_bezier_surface_item_EXPORTS
#  define SCENE_RATIONAL_BEZIER_SURFACE_ITEM_EXPORT Q_DECL_EXPORT
#else
#  define SCENE_RATIONAL_BEZIER_SURFACE_ITEM_EXPORT Q_DECL_IMPORT
#endif

#include <CGAL/Three/Viewer_interface.h>
#include <CGAL/Three/Scene_item_rendering_helper.h>

class dtkRationalBezierSurface;
struct Scene_rational_bezier_surface_item_priv;

class SCENE_RATIONAL_BEZIER_SURFACE_ITEM_EXPORT Scene_rational_bezier_surface_item
    :public CGAL::Three::Scene_item_rendering_helper
{
   Q_OBJECT
public:
   Scene_rational_bezier_surface_item(const dtkRationalBezierSurface& dtk_rational_bezier_surface);
  ~Scene_rational_bezier_surface_item();
   void initializeBuffers(CGAL::Three::Viewer_interface *) const Q_DECL_OVERRIDE;
  bool isEmpty() const Q_DECL_OVERRIDE;
  void draw(CGAL::Three::Viewer_interface *) const Q_DECL_OVERRIDE;
  Scene_item* clone() const Q_DECL_OVERRIDE {return 0;}
  QString toolTip() const Q_DECL_OVERRIDE;
  bool supportsRenderingMode(RenderingMode m) const Q_DECL_OVERRIDE { return (m == Flat || m == FlatPlusEdges); }
  void compute_bbox() const Q_DECL_OVERRIDE;
  QMenu* contextMenu() Q_DECL_OVERRIDE;
  void computeElements() const Q_DECL_OVERRIDE;

protected:
  friend struct Scene_rational_bezier_surface_item_priv;
  Scene_rational_bezier_surface_item_priv* d;
};

#endif // SCENE_RATIONAL_BEZIER_SURFACE_ITEM_H
