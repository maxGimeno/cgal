#include <CGAL/Mesh_3/io_signature.h>
#include "Scene_c3t3_cad_item.h"
#include "Kernel_type.h"
#include <CGAL/Mesh_3/tet_soup_to_c3t3.h>
#include <CGAL/Three/Polyhedron_demo_io_plugin_interface.h>
#include <CGAL/Three/Polyhedron_demo_plugin_interface.h>
#include <CGAL/IO/File_avizo.h>
#include <iostream>
#include <fstream>

#include <QMessageBox>

class Polyhedron_demo_c3t3_binary_io_plugin :
  public QObject,
  public CGAL::Three::Polyhedron_demo_io_plugin_interface,
  public CGAL::Three::Polyhedron_demo_plugin_interface
{
    Q_OBJECT
    Q_INTERFACES(CGAL::Three::Polyhedron_demo_io_plugin_interface)
    Q_INTERFACES(CGAL::Three::Polyhedron_demo_plugin_interface)
    Q_PLUGIN_METADATA(IID "com.geometryfactory.PolyhedronDemo.PluginInterface/1.0")

public:
  void init(QMainWindow*, CGAL::Three::Scene_interface* sc, Messages_interface*) override
  {
    this->scene = sc;
  }
  QString name() const override { return "Cad_c3t3_io_plugin"; }
  QString nameFilters() const override { return "cad binary files (*.cgal)"; }
  QString saveNameFilters() const override { return "cad binary files (*.cgal)"; }
  QString loadNameFilters() const override { return "cad binary files (*.cgal)"; }
  QList<QAction*> actions() const override
  {
    return QList<QAction*>();
  }
  bool applicable(QAction*) const override
  {
    return false;
  }
  bool canLoad(QFileInfo) const override {return false; }
  QList<CGAL::Three::Scene_item*> load(QFileInfo , bool&, bool) override {
    return {};
  }

  bool canSave(const CGAL::Three::Scene_item*) override;
  bool save(const CGAL::Three::Scene_item*, QFileInfo fileinfo);
  bool save(QFileInfo fileinfo,QList<CGAL::Three::Scene_item*>& items) override{
    Scene_item* item = items.front();
    if(save(item, fileinfo)) {
      items.pop_front();
      return true;
    } else {
      return false;
    }
  };

private:
  CGAL::Three::Scene_interface* scene;
};

bool Polyhedron_demo_c3t3_binary_io_plugin::canSave(const CGAL::Three::Scene_item* item)
{
  // This plugin supports c3t3 items.
  return qobject_cast<const Scene_c3t3_cad_item*>(item);
}

bool
Polyhedron_demo_c3t3_binary_io_plugin::
save(const CGAL::Three::Scene_item* item, QFileInfo fileinfo)
{
    const Scene_c3t3_cad_item* c3t3_item = qobject_cast<const Scene_c3t3_cad_item*>(item);
    if ( NULL == c3t3_item )
    {
      return false;
    }

    QString path = fileinfo.absoluteFilePath();

    if(path.endsWith(".cgal"))
    {
    std::ofstream out(fileinfo.filePath().toUtf8(),
                     std::ios_base::out|std::ios_base::binary);

    return out && c3t3_item->save_binary(out);
    }
    else
        return false;
}

struct Fake_mesh_domain {
  typedef CGAL::Tag_true Has_features;
  typedef int Subdomain_index;
  typedef std::pair<int,int> Surface_patch_index;
  typedef int Curve_segment_index;
  typedef int Corner_index;
  typedef boost::variant<Subdomain_index,Surface_patch_index> Index;
};

typedef Geom_traits_cad Fake_gt;
typedef CGAL::Mesh_vertex_base_3<Fake_gt, Fake_mesh_domain> Fake_vertex_base;
typedef CGAL::Compact_mesh_cell_base_3<Fake_gt, Fake_mesh_domain> Fake_cell_base;
typedef CGAL::Triangulation_data_structure_3<Fake_vertex_base,Fake_cell_base> Fake_tds;
typedef CGAL::Regular_triangulation_3<Fake_gt, Fake_tds> Fake_tr;
typedef CGAL::Mesh_complex_3_in_triangulation_3<
  Fake_tr,
  Fake_mesh_domain::Corner_index,
  Fake_mesh_domain::Curve_segment_index> Fake_c3t3;

template <class Vb = CGAL::Triangulation_vertex_base_3<Kernel> >
struct Fake_CDT_3_vertex_base : public Vb
{
  typedef Vb Base;
  bool steiner;
  std::size_t ref_1, ref_2;

  template < typename TDS2 >
  struct Rebind_TDS {
    typedef typename Base::template Rebind_TDS<TDS2>::Other   Vb2;
    typedef Fake_CDT_3_vertex_base<Vb2>  Other;
  };
};

template <class Vb>
std::istream&
operator>>( std::istream& is, Fake_CDT_3_vertex_base<Vb>& v)
{
  is >> static_cast<typename Fake_CDT_3_vertex_base<Vb>::Base&>(v);
  char s;
  if( CGAL::is_ascii(is) ) {
    is >> s;
    if( s == 'S' ) {
      v.steiner = true;
      is >> v.ref_1 >> v.ref_2;
    }
    else {
      CGAL_assertion(s == '.' || s == 'F');
      v.steiner = false;
    }
  } else {
    CGAL::read( is, s );
    if(is.bad()) return is;
    if( s == 'S' ) {
      v.steiner = true;
      CGAL::read( is, v.ref_1 );
      CGAL::read( is, v.ref_2 );
    }
    else {
      // if(s != '.') {
      // 	std::cerr << "v.point()=" << v.point() << std::endl;
      // 	std::cerr << "s=" << s << " (" << (int)s
      // 		  << "), just before position "
      // 		  << is.tellg() << " !\n";
      // }
      CGAL_assertion(s == '.' || s== 'F');
      v.steiner = false;
    }
  }
  return is;
}

template <class Cb = CGAL::Triangulation_cell_base_3<Kernel> >
struct Fake_CDT_3_cell_base : public Cb
{
  typedef Cb Base;
  int constrained_facet[4];
  bool _restoring[6];
  int to_edge_index( int li, int lj ) const {
    CGAL_triangulation_precondition( li >= 0 && li < 4 );
    CGAL_triangulation_precondition( lj >= 0 && lj < 4 );
    CGAL_triangulation_precondition( li != lj );
    return ( li==0 || lj==0 ) ? li+lj-1 : li+lj;
  }

  template < typename TDS2 >
  struct Rebind_TDS {
    typedef typename Base::template Rebind_TDS<TDS2>::Other   Cb2;
    typedef Fake_CDT_3_cell_base<Cb2>  Other;
  };
};

template <typename Cb>
std::istream&
operator>>( std::istream& is, Fake_CDT_3_cell_base<Cb>& c) {
  char s;
  for( int li = 0; li < 4; ++li ) {
    if( CGAL::is_ascii(is) )
      is >> c.constrained_facet[li];
    else
      CGAL::read( is, c.constrained_facet[li] );
  }

  if( CGAL::is_ascii(is) ) {
    is >> s;
    CGAL_assertion(s == '-');
  }
  is >> static_cast<typename Fake_CDT_3_cell_base<Cb>::Base&>(c);
  for( int li = 0; li < 3; ++li ) {
    for( int lj = li+1; lj < 4; ++lj ) {
      char s;
      is >> s;
      if(s == 'C') {
        c._restoring[c.to_edge_index(li, lj)] = true;
      } else {
        if(s != '.') {
          std::cerr << "cDT cell:";
          for( int li = 0; li < 4; ++li ) {
            std::cerr << " " << c.constrained_facet[li];
          }
          std::cerr << "\n";
          std::cerr << "s=" << s << " (" << (int)s
                    << "), just before position "
                    << is.tellg() << " !\n";	}
        CGAL_assertion(s == '.');
        c._restoring[c.to_edge_index(li, lj)] = false;
      }
    }
  }
  return is;
}

typedef CGAL::Triangulation_data_structure_3<Fake_CDT_3_vertex_base<>, Fake_CDT_3_cell_base<> > Fake_CDT_3_TDS;
typedef CGAL::Triangulation_3<Kernel, Fake_CDT_3_TDS> Fake_CDT_3;

typedef Fake_mesh_domain::Surface_patch_index Fake_patch_id;

template <typename Tr1, typename Tr2>
struct Update_vertex
{
  typedef Fake_mesh_domain::Surface_patch_index Sp_index;
  typedef typename Tr1::Vertex                  V1;
  typedef typename Tr2::Vertex                  V2;
  typedef typename Tr2::Point                   Point;

  bool operator()(const V1& v1, V2& v2)
  {
    v2.set_point(Point(v1.point()));
    v2.set_dimension(v1.in_dimension());
    v2.set_special(v1.is_special());
    switch(v1.in_dimension()) {
    case 2:
    {
      const typename V1::Index& index = v1.index();
      const Sp_index sp_index = boost::get<Sp_index>(index);
      v2.set_index((std::max)(sp_index.first, sp_index.second));
    }
    break;
    default:// -1, 0, 1, 3
      v2.set_index(boost::get<int>(v1.index()));
    }
    return true;
  }
}; // end struct Update_vertex

struct Update_cell {
  typedef Fake_mesh_domain::Surface_patch_index Sp_index;
  template <typename C1, typename C2>
  bool operator()(const C1& c1, C2& c2) {
    c2.set_subdomain_index(c1.subdomain_index());
    for(int i = 0; i < 4; ++i) {
      const Sp_index sp_index = c1.surface_patch_index(i);
      c2.set_surface_patch_index(i, std::make_pair(sp_index.first,
                                               sp_index.second));
      CGAL_assertion(c1.is_facet_on_surface(i) ==
                     c2.is_facet_on_surface(i));
    }
    return true;
  }
}; // end struct Update_cell




#include <QtPlugin>
#include "Cad_c3t3_io_plugin.moc"
