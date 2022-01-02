#include <stdio.h>
#include <string.h>
#include <zlib.h>

/***********************************************
Reads the first line from one or many .gz files and writes that to an
output file (in append mode). Input is the name of a file that contains
a list of .gz files.

This is designed to read an NOS manifest file of xyz.gz files. The first
line of each .xyz file is a description of the columns in the file (an
xyz file has csv format)

***********************************************/

int main(int argc, char **argv) 
{
   int rc = 1;
   if (argc != 3) {
      printf("Usage: %s <input file> <output file>\n", argv[0]);
      printf("where\n");
      printf("      <input file> is a text file containing input gz "
            "files to read\n");
      printf("      <output file> is file where summary data is stored "
            "(opened in append mode)\n");
      return 1;
   }
   const char *in_file = argv[1];
   const char *out_file = argv[2];
   char buf[256];
   char buf2[256];
   FILE *ofp = NULL;
   FILE *ifp = NULL;
   ifp = fopen(in_file, "r");
   if (!ifp) {
      fprintf(stderr, "Unable to open input file '%s'\n", in_file);
      goto end;
   }
   ofp = fopen(out_file, "a");
   if (!ofp) {
      fprintf(stderr, "Unable to open output file '%s'\n", out_file);
      goto end;
   }
   printf("Opened '%s' for writing\n", out_file);
   //
   while (fgets(buf, sizeof(buf), ifp)) {
      // strip trailing newline
      size_t sz = strlen(buf);
      if (sz <= 1) {
         continue;
      }
      buf[sz-1] = 0;
      gzFile gz_in = NULL;
      gz_in = gzopen(buf, "rb");
      if (!gz_in) {
         fprintf(stderr, "Unable to open gz file '%s'\n", buf);
         continue;
      }
      // read header line
      gzgets(gz_in, buf2, sizeof(buf2));
      fprintf(ofp, "%s\t%s", buf, buf2);
      printf("%s\t%s", buf, buf2);
      gzclose(gz_in);
   }
   rc = 0;
end:
   if (ifp) {
      fclose(ifp);
   }
   if (ofp) {
      fclose(ofp);
   }
   return rc;
}

