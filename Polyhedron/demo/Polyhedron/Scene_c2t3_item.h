#ifndef SCENE_C2T3_ITEM_H
#define SCENE_C2T3_ITEM_H

#include "Scene_c2t3_item_config.h"
#include "C2t3_type.h"
#include <iostream>
#include  <CGAL/Three/Scene_item.h>
#include <qgl.h>
#include <QtCore/qglobal.h>
#include <CGAL/gl.h>

class SCENE_C2T3_ITEM_EXPORT Scene_c2t3_item : public CGAL::Three::Scene_item
{
  Q_OBJECT
public:
  Scene_c2t3_item(const C2t3& c2t3)
    : c2t3_(c2t3)
  {
  }

  ~Scene_c2t3_item()
  {
  }

  C2t3& c2t3() {
    return c2t3_;
  }

  const C2t3& c2t3() const {
    return c2t3_;
  }
  bool isFinite() const { return true; }
  bool isEmpty() const {
    return c2t3().triangulation().number_of_vertices() == 0;
  }

  void compute_bbox() const {
    if(isEmpty())
      _bbox =  Bbox();
    else {
      bool first = true;
      CGAL::Bbox_3 result;
      for(Tr::Finite_vertices_iterator
            vit = ++c2t3().triangulation().finite_vertices_begin(),
            end = c2t3().triangulation().finite_vertices_end();
          vit != end; ++vit)
      {
        if(first) {
          result = vit->point().bbox();
          first = false;
        } else { 
          result = result + vit->point().bbox();
        }
      }
      _bbox = Bbox(result.xmin(), result.ymin(), result.zmin(),
                  result.xmax(), result.ymax(), result.zmax());
    }
  }


  Scene_c2t3_item* clone() const {
    return 0;
  }

  QString toolTip() const {
    return tr("<p><b>2D complex in a 3D triangulation</b></p>"
              "<p>Number of vertices: %1<br />"
              "Number of surface facets: %2<br />")
      .arg(c2t3().triangulation().number_of_vertices())
      .arg(c2t3().number_of_facets());
  }

  // Indicate if rendering mode is supported
  bool supportsRenderingMode(RenderingMode m) const {
    return (m != Gouraud && m!=PointsPlusNormals && m!=Splatting); // CHECK THIS!
  }

  void draw() const {

  }

private:
  C2t3 c2t3_;
};

#endif // SCENE_C2T3_ITEM
