/* Copyright  (C) 2010-2017 The RetroArch team
 *
 * ---------------------------------------------------------------------------------------
 * The following license statement only applies to this file (libretrodb_tool.c).
 * ---------------------------------------------------------------------------------------
 *
 * Permission is hereby granted, free of charge,
 * to any person obtaining a copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software,
 * and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 * INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include <stdio.h>
#include <string.h>

#include "stdstring.h"

#include "libretrodb.h"
#include "rmsgpack_dom.h"

int libretro_tool_main(int argc, char ** argv)
{
   int rv;
   struct rmsgpack_dom_value item;
   const char *command                            = NULL;
   const char *path                               = NULL;
   const char *query_exp                          = NULL;
   const char *error                              = NULL;
   libretrodb_t *db                               = NULL;
   libretrodb_cursor_t *cur                       = NULL;
   libretrodb_query_t *q                          = NULL;


   command = "find";//argv[2];
   path    = "/dd64_firmware/N64.rdb";//argv[1];

   db      = libretrodb_new();
   cur     = libretrodb_cursor_new();

   if (!db || !cur)
      goto error;

   if ((rv = libretrodb_open(path, db)) != 0)
   {
      printf("Could not open db file '%s'\n", path);
      goto error;
   }

   query_exp = "{'serial':b'4E474545'}";//argv[3];
   error = NULL;
   q = libretrodb_query_compile(db, query_exp, strlen(query_exp), &error);

   if (error)
   {
      printf("%s\n", error);
      goto error;
   }

   if ((rv = libretrodb_cursor_open(db, cur, q)) != 0)
   {
      printf("Could not open cursor\n");
      goto error;
   }

   while (libretrodb_cursor_read_item(cur, &item) == 0)
   {
      rmsgpack_dom_value_print(&item);
      printf("\n");
      rmsgpack_dom_value_free(&item);
      break;
   }

   libretrodb_cursor_close(cur);
   libretrodb_close(db);

error:
   if (db)
      libretrodb_free(db);
   if (cur)
      libretrodb_cursor_free(cur);
   if (q)
      libretrodb_query_free(q);
   return 1;
}
