import shapefile     # from the package pyshp

#dataset = "l"     # low resolution ~1M
#dataset = "i"     # intermediate resolution ~7M
#dataset = "h"     # high resolution ~34M
dataset = "f"     # full resolution ~200M

fname = '/data/mapping/shoreline/GSHHS_shp/{}/GSHHS_{}_L1.shp'.format(dataset, dataset)

sf = shapefile.Reader(fname)

#border = 2.5
#
#def export_box(l, r, t, b):
#   left_limit = l - border
#   right_limit = r + border
#   top_limit = t + border
#   bottom_limit = b - border
#   name = "box_{}_{}.txt".format(l, t)

name = "box_{}.txt".format(dataset)
with open(name, 'w') as f:
   for shape in list(sf.iterShapes()):
      npoints=len(shape.points) # total points
      nparts = len(shape.parts) # total parts
      #
      if nparts == 1:
         x = shape.points[0][0]
         y = shape.points[0][1]
         for p in shape.points:
            f.write("{} {}\n".format(p[0], p[1]))
         f.write("#\n")
      else: # loop over parts of each shape, plot separately
         for ip in range(nparts): # loop over parts, plot separately
            i0=shape.parts[ip]
            if ip < nparts-1:
               i1 = shape.parts[ip+1]-1
            else:
               i1 = npoints
            seg=shape.points[i0:i1+1]
            x = seg[0][0]
            y = seg[0][1]
            for p in seg:
               f.write("{} {}\n".format(p[0], p[1]))
            f.write("#\n")

