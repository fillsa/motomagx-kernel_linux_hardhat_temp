/* 
 * xfrm4_policy.c
 *
 * Changes:
 *	Kazunori MIYAZAWA @USAGI
 * 	YOSHIFUJI Hideaki @USAGI
 *		Split up af-specific portion
 *
 *
 * Copyright (C) 2007,2008 Motorola Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 * 
 * Date         Author          Comment
 * 04/2007      Motorola        UMA DSCP support - new funcs added
 * 03/2008      Motorola        Fixed segfault in UMA DSCP code
 */

#include <linux/config.h>
#include <net/xfrm.h>
#include <net/ip.h>

#ifdef CONFIG_MOT_FEAT_DSCP_UMA
#include <net/inet_ecn.h>
#endif

static struct dst_ops xfrm4_dst_ops;
static struct xfrm_policy_afinfo xfrm4_policy_afinfo;

static struct xfrm_type_map xfrm4_type_map = { .lock = RW_LOCK_UNLOCKED };

static int xfrm4_dst_lookup(struct xfrm_dst **dst, struct flowi *fl)
{
	return __ip_route_output_key((struct rtable**)dst, fl);
}

/* Check that the bundle accepts the flow and its components are
 * still valid.
 */

static int __xfrm4_bundle_ok(struct xfrm_dst *xdst, struct flowi *fl)
{
	do {
		if (xdst->u.dst.ops != &xfrm4_dst_ops)
			return 1;

		if (!xfrm_selector_match(&xdst->u.dst.xfrm->sel, fl, AF_INET))
			return 0;
		if (xdst->u.dst.xfrm->km.state != XFRM_STATE_VALID ||
		    xdst->u.dst.path->obsolete > 0)
			return 0;
		xdst = (struct xfrm_dst*)xdst->u.dst.child;
	} while (xdst);
	return 0;
}

static struct dst_entry *
__xfrm4_find_bundle(struct flowi *fl, struct xfrm_policy *policy)
{
	struct dst_entry *dst;

	read_lock_bh(&policy->lock);
	for (dst = policy->bundles; dst; dst = dst->next) {
		struct xfrm_dst *xdst = (struct xfrm_dst*)dst;
		if (xdst->u.rt.fl.oif == fl->oif &&	/*XXX*/
		    xdst->u.rt.fl.fl4_dst == fl->fl4_dst &&
	    	    xdst->u.rt.fl.fl4_src == fl->fl4_src &&
		    __xfrm4_bundle_ok(xdst, fl)) {
			dst_clone(dst);
			break;
		}
	}
	read_unlock_bh(&policy->lock);
	return dst;
}

/* Allocate chain of dst_entry's, attach known xfrm's, calculate
 * all the metrics... Shortly, bundle a bundle.
 */

static int
__xfrm4_bundle_create(struct xfrm_policy *policy, struct xfrm_state **xfrm, int nx,
		      struct flowi *fl, struct dst_entry **dst_p)
{
	struct dst_entry *dst, *dst_prev;
	struct rtable *rt0 = (struct rtable*)(*dst_p);
	struct rtable *rt = rt0;
	u32 remote = fl->fl4_dst;
	u32 local  = fl->fl4_src;
	int i;
	int err;
	int header_len = 0;
	int trailer_len = 0;

	dst = dst_prev = NULL;

	for (i = 0; i < nx; i++) {
		struct dst_entry *dst1 = dst_alloc(&xfrm4_dst_ops);

		if (unlikely(dst1 == NULL)) {
			err = -ENOBUFS;
			goto error;
		}

		if (!dst)
			dst = dst1;
		else {
			dst_prev->child = dst1;
			dst1->flags |= DST_NOHASH;
			dst_clone(dst1);
		}
		dst_prev = dst1;
		if (xfrm[i]->props.mode) {
			remote = xfrm[i]->id.daddr.a4;
			local  = xfrm[i]->props.saddr.a4;
		}
		header_len += xfrm[i]->props.header_len;
		trailer_len += xfrm[i]->props.trailer_len;
	}

	if (remote != fl->fl4_dst) {
		struct flowi fl_tunnel = { .nl_u = { .ip4_u =
						     { .daddr = remote,
						       .saddr = local }
					           }
				         };
		err = xfrm_dst_lookup((struct xfrm_dst**)&rt, &fl_tunnel, AF_INET);
		if (err)
			goto error;
	} else {
		dst_hold(&rt->u.dst);
	}
	dst_prev->child = &rt->u.dst;
	i = 0;
	for (dst_prev = dst; dst_prev != &rt->u.dst; dst_prev = dst_prev->child) {
		struct xfrm_dst *x = (struct xfrm_dst*)dst_prev;
		x->u.rt.fl = *fl;

		dst_prev->xfrm = xfrm[i++];
		dst_prev->dev = rt->u.dst.dev;
		if (rt->u.dst.dev)
			dev_hold(rt->u.dst.dev);
		dst_prev->obsolete	= -1;
		dst_prev->flags	       |= DST_HOST;
		dst_prev->lastuse	= jiffies;
		dst_prev->header_len	= header_len;
		dst_prev->trailer_len	= trailer_len;
		memcpy(&dst_prev->metrics, &rt->u.dst.metrics, sizeof(dst_prev->metrics));
		dst_prev->path		= &rt->u.dst;

		/* Copy neighbout for reachability confirmation */
		dst_prev->neighbour	= neigh_clone(rt->u.dst.neighbour);
		dst_prev->input		= rt->u.dst.input;
		dst_prev->output	= xfrm4_output;
		if (rt->peer)
			atomic_inc(&rt->peer->refcnt);
		x->u.rt.peer = rt->peer;
		/* Sheit... I remember I did this right. Apparently,
		 * it was magically lost, so this code needs audit */
		x->u.rt.rt_flags = rt0->rt_flags&(RTCF_BROADCAST|RTCF_MULTICAST|RTCF_LOCAL);
		x->u.rt.rt_type = rt->rt_type;
		x->u.rt.rt_src = rt0->rt_src;
		x->u.rt.rt_dst = rt0->rt_dst;
		x->u.rt.rt_gateway = rt->rt_gateway;
		x->u.rt.rt_spec_dst = rt0->rt_spec_dst;
		header_len -= x->u.dst.xfrm->props.header_len;
		trailer_len -= x->u.dst.xfrm->props.trailer_len;
	}
	*dst_p = dst;
	return 0;

error:
	if (dst)
		dst_free(dst);
	return err;
}

static void
_decode_session4(struct sk_buff *skb, struct flowi *fl)
{
	struct iphdr *iph = skb->nh.iph;
	u8 *xprth = skb->nh.raw + iph->ihl*4;

	memset(fl, 0, sizeof(struct flowi));
	if (!(iph->frag_off & htons(IP_MF | IP_OFFSET))) {
		switch (iph->protocol) {
		case IPPROTO_UDP:
		case IPPROTO_TCP:
		case IPPROTO_SCTP:
			if (pskb_may_pull(skb, xprth + 4 - skb->data)) {
				u16 *ports = (u16 *)xprth;

				fl->fl_ip_sport = ports[0];
				fl->fl_ip_dport = ports[1];
			}
			break;

		case IPPROTO_ICMP:
			if (pskb_may_pull(skb, xprth + 2 - skb->data)) {
				u8 *icmp = xprth;

				fl->fl_icmp_type = icmp[0];
				fl->fl_icmp_code = icmp[1];
			}
			break;

		case IPPROTO_ESP:
			if (pskb_may_pull(skb, xprth + 4 - skb->data)) {
				u32 *ehdr = (u32 *)xprth;

				fl->fl_ipsec_spi = ehdr[0];
			}
			break;

		case IPPROTO_AH:
			if (pskb_may_pull(skb, xprth + 8 - skb->data)) {
				u32 *ah_hdr = (u32*)xprth;

				fl->fl_ipsec_spi = ah_hdr[1];
			}
			break;

		case IPPROTO_COMP:
			if (pskb_may_pull(skb, xprth + 4 - skb->data)) {
				u16 *ipcomp_hdr = (u16 *)xprth;

				fl->fl_ipsec_spi = ntohl(ntohs(ipcomp_hdr[1]));
			}
			break;
		default:
			fl->fl_ipsec_spi = 0;
			break;
		};
	}
	fl->proto = iph->protocol;
	fl->fl4_dst = iph->daddr;
	fl->fl4_src = iph->saddr;
}

#ifdef CONFIG_MOT_FEAT_DSCP_UMA
void xfrm4_copy_dscp(struct iphdr *outer, struct iphdr *inner)
{
    struct flowi fl;
    struct xfrm_policy* policy = NULL;

    if((inner !=NULL)&&(outer != NULL))
    {
        fl.fl4_dst = inner->daddr;
        fl.fl4_src = inner->saddr;
    
        policy = flow_cache_lookup(&fl, AF_INET, FLOW_DIR_IN, xfrm_policy_lookup);

        // Extract the dscp field (first 6 bits of the TOS byte) from the outer ip
        // header. Put it in the dscp array of the SP at the index that corresponds 
        // to the dscp field of the inner ip header.
        if (policy != NULL)
        {
            policy->dscp[(inner->tos & ~INET_ECN_MASK)>>2] = (outer->tos & ~INET_ECN_MASK)>>2;
        }
    }
}

void xfrm4_extract_dscp(struct iphdr *outer, struct iphdr *inner)
{
    struct flowi fl;
    struct xfrm_policy* policy = NULL;
    
    if((inner !=NULL)&&(outer!=NULL))
    {
        fl.fl4_dst = inner->saddr;
        fl.fl4_src = inner->daddr;

        policy = flow_cache_lookup(&fl, AF_INET, FLOW_DIR_IN, xfrm_policy_lookup);

        // The tos field is already copied from inner to outer header before this
        // function is called from xfrm4_encap function.

        // Take the tos field from the inner (which is same as outer) and extract
        // dscp field. This gives the index into the dscp array of the SP.
        // Extract the dscp field from this array and OR it with the ECN mask.
        // Then put this value into the TOS field of the outer ip header.
        if (policy != NULL)
        {
            outer->tos = (policy->dscp[outer->tos >> 2])<<2 | (outer->tos & INET_ECN_MASK);
        }
    }	
}
#endif

static inline int xfrm4_garbage_collect(void)
{
	read_lock(&xfrm4_policy_afinfo.lock);
	xfrm4_policy_afinfo.garbage_collect();
	read_unlock(&xfrm4_policy_afinfo.lock);
	return (atomic_read(&xfrm4_dst_ops.entries) > xfrm4_dst_ops.gc_thresh*2);
}

static void xfrm4_update_pmtu(struct dst_entry *dst, u32 mtu)
{
	struct dst_entry *path = dst->path;

	if (mtu < 68 + dst->header_len)
		return;

	path->ops->update_pmtu(path, mtu);
}

static struct dst_ops xfrm4_dst_ops = {
	.family =		AF_INET,
	.protocol =		__constant_htons(ETH_P_IP),
	.gc =			xfrm4_garbage_collect,
	.update_pmtu =		xfrm4_update_pmtu,
	.gc_thresh =		1024,
	.entry_size =		sizeof(struct xfrm_dst),
};

static struct xfrm_policy_afinfo xfrm4_policy_afinfo = {
	.family = 		AF_INET,
	.lock = 		RW_LOCK_UNLOCKED,
	.type_map = 		&xfrm4_type_map,
	.dst_ops =		&xfrm4_dst_ops,
	.dst_lookup =		xfrm4_dst_lookup,
	.find_bundle = 		__xfrm4_find_bundle,
	.bundle_create =	__xfrm4_bundle_create,
	.decode_session =	_decode_session4,
};

static void __init xfrm4_policy_init(void)
{
	xfrm_policy_register_afinfo(&xfrm4_policy_afinfo);
}

static void __exit xfrm4_policy_fini(void)
{
	xfrm_policy_unregister_afinfo(&xfrm4_policy_afinfo);
}

void __init xfrm4_init(void)
{
	xfrm4_state_init();
	xfrm4_policy_init();
}

void __exit xfrm4_fini(void)
{
	//xfrm4_input_fini();
	xfrm4_policy_fini();
	xfrm4_state_fini();
}

