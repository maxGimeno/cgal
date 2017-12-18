#ifndef SCENE_RATIONAL_BEZIER_CURVE_ITEM_H
#define SCENE_RATIONAL_BEZIER_CURVE_ITEM_H


#ifdef scene_rational_bezier_curve_item_EXPORTS
#  define SCENE_RATIONAL_BEZIER_CURVE_ITEM_EXPORT Q_DECL_EXPORT
#else
#  define SCENE_RATIONAL_BEZIER_CURVE_ITEM_EXPORT Q_DECL_IMPORT
#endif

#include <CGAL/Three/Viewer_interface.h>
#include <CGAL/Three/Scene_item.h>

class dtkRationalBezierCurve;
struct Scene_rational_bezier_curve_item_priv;

class SCENE_RATIONAL_BEZIER_CURVE_ITEM_EXPORT Scene_rational_bezier_curve_item
    :public CGAL::Three::Scene_item
{
   Q_OBJECT
public:
   Scene_rational_bezier_curve_item(const dtkRationalBezierCurve& dtk_rational_bezier_curve);
  ~Scene_rational_bezier_curve_item();
  bool isEmpty() const;
  void draw(CGAL::Three::Viewer_interface *) const;
  Scene_item* clone() const {return 0;}
  QString toolTip() const {return QString();}
  bool supportsRenderingMode(RenderingMode m) const { return (m == Flat || m == FlatPlusEdges || m == Wireframe); }
  QMenu* contextMenu();

protected:
  friend struct Scene_rational_bezier_curve_item_priv;
  Scene_rational_bezier_curve_item_priv* d;
};

#endif // SCENE_RATIONAL_BEZIER_CURVE_ITEM_H
