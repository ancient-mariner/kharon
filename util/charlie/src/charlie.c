#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include "pinet.h"
#include "postmaster.h"
#include "dev_info.h"

#include "charlie.h"

////////////////////////////////////////////////////////////////////////
// communicate with postamster

int get_postmaster_address(
      /*    out */       network_id_type *id
      )
{
   int rc = 1;
   // if getenv() returns NULL then host will default to value from
   //    gethostname()
   char *host = getenv(CHARLIE_HOST_STR);
   if (resolve_endpoint_for_host("postmaster", "nonsensor", host, id) != 0) {
      fprintf(stderr, "Failed to get postmaster endpoint. Check "
            "dev/<host>/nonsensor\n");
      goto err;
   }
   rc = 0;
err:
   return rc;
}


int send_postmaster_request(
      /* in     */ const int sockfd, 
      /* in     */ const struct pm_request *req, 
      /* in     */ const void *data
      )
{
   // receive each field from pm_request in the order that it's defined
   uint32_t val;
   // read fields one-by-one
   val = htonl(req->request_type);
   if (send_block(sockfd, &val, sizeof(val)) < 0)
      goto err;
   val = htonl(req->header_bytes);
   if (send_block(sockfd, &val, sizeof(val)) < 0)
      goto err;
   val = htonl((uint32_t) req->custom_0);
   if (send_block(sockfd, &val, sizeof(val)) < 0)
      goto err;
   val = htonl((uint32_t) req->custom_1);
   if (send_block(sockfd, &val, sizeof(val)) < 0)
      goto err;
   val = htonl((uint32_t) req->custom_2);
   if (send_block(sockfd, &val, sizeof(val)) < 0)
      goto err;
   if (req->header_bytes > 0) {
      if (send_block(sockfd, data, req->header_bytes) != 
            (int32_t) req->header_bytes) {
         fprintf(stderr, "Problem sending request data (%d bytes)\n",
               req->header_bytes);
         goto err;
      }
   }
   return 0;
err:
   fprintf(stderr, "Error reading PM request packet (reported type is %d)\n",
         req->request_type);
   return -1;
}


// if postmaster returns any extra data (indicated by response_bytes) then
//    memory is allocated and a pointer returned in *data. this must be freed
//    by the calling function
int read_postmaster_response(
      /* in     */ const int sockfd,
      /* in     */ const struct pm_request *req,
      /*    out */       struct pm_response *resp, 
      /*    out */       uint8_t ** data
      )
{
   // send each field from pm_response in the order that it's listed
   uint32_t val;
   uint32_t T_BUF_LEN = 32u;
   if (recv_block(sockfd, &val, sizeof(val)) < 0) {
      goto err;
   }
   resp->request_type = htonl(val);
   //
   if (recv_block(sockfd, &val, sizeof(val)) < 0) {
      goto err;
   }
   resp->response_bytes = htonl(val);
   //
   if (recv_block(sockfd, resp->t, T_BUF_LEN) < 0) {
      goto err;
   }
   //
   if (recv_block(sockfd, &val, sizeof(val)) < 0) {
      goto err;
   }
   resp->custom_0 = (int32_t) htonl(val);
   //
   if (recv_block(sockfd, &val, sizeof(val)) < 0) {
      goto err;
   }
   resp->custom_1 = (int32_t) htonl(val);
   //
   if (recv_block(sockfd, &val, sizeof(val)) < 0) {
      goto err;
   }
   resp->custom_2 = (int32_t) htonl(val);
   //
   if (resp->response_bytes > 0) {
      // allocate memory to fetch data
      *data = malloc(resp->response_bytes);
      if (recv_block(sockfd, *data, resp->response_bytes) != 
            (int32_t) resp->response_bytes) {
         fprintf(stderr, "Problem reading response data (%d bytes)\n",
               resp->response_bytes);
         goto err;
      }
   }
   return 0;
err:
   fprintf(stderr, "Error sending PM response packet (type = %d)\n",
         req->request_type);
   return -1;
}

// communicate with postamster
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
// 

