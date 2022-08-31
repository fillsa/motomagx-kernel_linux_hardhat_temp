/*
 *	Linux INET6 implementation
 *	FIB front-end.
 *
 *	Authors:
 *	Pedro Roque		<roque@di.fc.ul.pt>	
 *
 *	$Id: route.c,v 1.56 2001/10/31 21:55:55 davem Exp $
 *
 *	This program is free software; you can redistribute it and/or
 *      modify it under the terms of the GNU General Public License
 *      as published by the Free Software Foundation; either version
 *      2 of the License, or (at your option) any later version.
 */

/*	Changes:
 *
 *	YOSHIFUJI Hideaki @USAGI
 *		reworked default router selection.
 *		- respect outgoing interface
 *		- select from (probably) reachable routers (i.e.
 *		routers in REACHABLE, STALE, DELAY or PROBE states).
 *		- always select the same router if it is (probably)
 *		reachable.  otherwise, round-robin the list.
 * 	Ville Nuorvala
 *		Fixed routing subtrees.
 *		Moved source address selection to routing code.
 *		Implemented policy based routing.
 */

#include <linux/config.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/times.h>
#include <linux/socket.h>
#include <linux/sockios.h>
#include <linux/net.h>
#include <linux/route.h>
#include <linux/netdevice.h>
#include <linux/in6.h>
#include <linux/init.h>
#include <linux/netlink.h>
#include <linux/if_arp.h>

#ifdef 	CONFIG_PROC_FS
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#endif

#include <net/snmp.h>
#include <net/ipv6.h>
#include <net/ip6_fib.h>
#include <net/ip6_route.h>
#include <net/ndisc.h>
#include <net/addrconf.h>
#include <net/tcp.h>
#include <linux/rtnetlink.h>
#include <net/dst.h>
#include <net/xfrm.h>

#include <asm/uaccess.h>

#ifdef CONFIG_SYSCTL
#include <linux/sysctl.h>
#endif

/* Set to 3 to get tracing. */
#define RT6_DEBUG 2

#if RT6_DEBUG >= 3
#define RDBG(x) printk x
#define RT6_TRACE(x...) printk(KERN_DEBUG x)
#else
#define RDBG(x)
#define RT6_TRACE(x...) do { ; } while (0)
#endif


static int ip6_rt_max_size = 4096;
static int ip6_rt_gc_min_interval = HZ / 2;
static int ip6_rt_gc_timeout = 60*HZ;
int ip6_rt_gc_interval = 30*HZ;
static int ip6_rt_gc_elasticity = 9;
static int ip6_rt_mtu_expires = 10*60*HZ;
static int ip6_rt_min_advmss = IPV6_MIN_MTU - 20 - 40;

static struct rt6_info * ip6_rt_copy(struct rt6_info *ort);
static struct dst_entry	*ip6_dst_check(struct dst_entry *dst, u32 cookie);
static struct dst_entry *ip6_negative_advice(struct dst_entry *);
static void		ip6_dst_destroy(struct dst_entry *);
static void		ip6_dst_ifdown(struct dst_entry *,
				       struct net_device *dev, int how);
static int		 ip6_dst_gc(void);

static int		ip6_pkt_discard(struct sk_buff *skb);
static int		ip6_pkt_discard_out(struct sk_buff *skb);
static void		ip6_link_failure(struct sk_buff *skb);
static void		ip6_rt_update_pmtu(struct dst_entry *dst, u32 mtu);

static struct dst_ops ip6_dst_ops = {
	.family			=	AF_INET6,
	.protocol		=	__constant_htons(ETH_P_IPV6),
	.gc			=	ip6_dst_gc,
	.gc_thresh		=	1024,
	.check			=	ip6_dst_check,
	.destroy		=	ip6_dst_destroy,
	.ifdown			=	ip6_dst_ifdown,
	.negative_advice	=	ip6_negative_advice,
	.link_failure		=	ip6_link_failure,
	.update_pmtu		=	ip6_rt_update_pmtu,
	.entry_size		=	sizeof(struct rt6_info),
};

struct rt6_info ip6_null_entry = {
	.u = {
		.dst = {
			.__refcnt	= ATOMIC_INIT(1),
			.__use		= 1,
			.dev		= &loopback_dev,
			.obsolete	= -1,
			.error		= -ENETUNREACH,
			.metrics	= { [RTAX_HOPLIMIT - 1] = 255, },
			.input		= ip6_pkt_discard,
			.output		= ip6_pkt_discard_out,
			.ops		= &ip6_dst_ops,
			.path		= (struct dst_entry*)&ip6_null_entry,
		}
	},
	.rt6i_flags	= (RTF_REJECT | RTF_NONEXTHOP),
	.rt6i_metric	= ~(u32) 0,
	.rt6i_ref	= ATOMIC_INIT(1),
};

#ifdef CONFIG_IPV6_MULTIPLE_TABLES

struct rt6_info ip6_prohibit_entry = {
	.u = {
		.dst = {
			.__refcnt       = ATOMIC_INIT(1),
			.__use          = 1,
			.dev            = &loopback_dev,
			.obsolete       = -1,
			.error          = -EACCES,
			.metrics        = { [RTAX_HOPLIMIT - 1] = 255, },
			.input          = ip6_pkt_discard,
			.output         = ip6_pkt_discard_out,
			.ops            = &ip6_dst_ops,
			.path           = (struct dst_entry*)&ip6_prohibit_entry,
		}
	},
	.rt6i_flags     = (RTF_REJECT | RTF_NONEXTHOP),
	.rt6i_metric    = ~(u32) 0,
	.rt6i_ref       = ATOMIC_INIT(1),
};

struct rt6_info ip6_blk_hole_entry = {
	.u = {
		.dst = {
			.__refcnt       = ATOMIC_INIT(1),
			.__use          = 1,
			.dev            = &loopback_dev,
			.obsolete       = -1,
			.error          = -EINVAL,
			.metrics        = { [RTAX_HOPLIMIT - 1] = 255, },
			.input          = ip6_pkt_discard,
			.output         = ip6_pkt_discard_out,
			.ops            = &ip6_dst_ops,
			.path           = (struct dst_entry*)&ip6_blk_hole_entry,
		}
	},
	.rt6i_flags     = (RTF_REJECT | RTF_NONEXTHOP),
	.rt6i_metric    = ~(u32) 0,
	.rt6i_ref       = ATOMIC_INIT(1),
};

struct rt6_table *rt6_tables[RT6_TABLE_MAX + 1];

static struct rt6_table *rt6_tree_init(void)
{
	struct rt6_table *table;

	if ((table = kmalloc(sizeof(struct rt6_table), GFP_ATOMIC))) {
		memset(table, 0, sizeof(struct rt6_table));
		table->root.leaf = &ip6_null_entry;
		table->root.fn_flags = RTN_ROOT | RTN_TL_ROOT | RTN_RTINFO;
		rwlock_init(&table->lock);
	}
	return table;
}

struct rt6_table *__rt6_new_table(int id)
{
	struct rt6_table *table = rt6_tree_init();
	rt6_tables[id] = table;
	return table;
}

#else

struct rt6_table ip6_routing_table = {
	.lock           = RW_LOCK_UNLOCKED,
	.root           = { .leaf	= &ip6_null_entry,
			    .fn_flags	= RTN_ROOT | RTN_TL_ROOT | RTN_RTINFO,
	},
};

#endif

/* Write lock protects all the ip6 fibs, read lock and table->lock is enough
   to protect individual ip6 fibs */

DEFINE_RWLOCK(rt6_lock);

DEFINE_SPINLOCK(ndisc_lock);


/* allocate dst with ip6_dst_ops */
static __inline__ struct rt6_info *ip6_dst_alloc(void)
{
	return (struct rt6_info *)dst_alloc(&ip6_dst_ops);
}

static void ip6_dst_destroy(struct dst_entry *dst)
{
	struct rt6_info *rt = (struct rt6_info *)dst;
	struct inet6_dev *idev = rt->rt6i_idev;

	if (idev != NULL) {
		rt->rt6i_idev = NULL;
		in6_dev_put(idev);
	}	
}

static void ip6_dst_ifdown(struct dst_entry *dst, struct net_device *dev,
			   int how)
{
	struct rt6_info *rt = (struct rt6_info *)dst;
	struct inet6_dev *idev = rt->rt6i_idev;

	if (dev != &loopback_dev && idev != NULL && idev->dev == dev) {
		struct inet6_dev *loopback_idev = in6_dev_get(&loopback_dev);
		if (loopback_idev != NULL) {
			rt->rt6i_idev = loopback_idev;
			in6_dev_put(idev);
		}
	}
}

static __inline__ int ip6_rt_check_expired(struct rt6_info *rt)
{
	return ((rt->rt6i_flags & RTF_EXPIRES) &&
		time_after(jiffies, rt->rt6i_expires));
}

static __inline__ int rt6_check_expired(const struct rt6_info *rt)
{
	return (rt->rt6i_flags & RTF_EXPIRES &&
		time_after(jiffies, rt->rt6i_expires));
}

/*
 *	Route lookup. Any table->lock is implied.
 */

static __inline__ struct rt6_info *rt6_device_match(struct rt6_info *rt,
						    int oif,
						    int strict)
{
	struct rt6_info *local = NULL;
	struct rt6_info *sprt;

	if (oif) {
		for (sprt = rt; sprt; sprt = sprt->u.next) {
			struct net_device *dev = sprt->rt6i_dev;
			if (dev->ifindex == oif)
				return sprt;
			if (dev->flags & IFF_LOOPBACK)
				local = sprt;
		}

		if (local)
			return local;

		if (strict)
			return &ip6_null_entry;
	}
	return rt;
}

/*
 *	pointer to the last default router chosen. BH is disabled locally.
 */
static struct rt6_info *rt6_dflt_pointer;
static DEFINE_SPINLOCK(rt6_dflt_lock);

void rt6_reset_dflt_pointer(struct rt6_info *rt)
{
	spin_lock_bh(&rt6_dflt_lock);
	if (rt == NULL || rt == rt6_dflt_pointer) {
		RT6_TRACE("reset default router: %p->NULL\n", rt6_dflt_pointer);
		rt6_dflt_pointer = NULL;
	}
	spin_unlock_bh(&rt6_dflt_lock);
}

/* Default Router Selection (RFC 2461 6.3.6) */
static struct rt6_info *rt6_best_dflt(struct rt6_table *table,
				      struct rt6_info *rt, int oif)
{
	struct rt6_info *match = NULL;
	struct rt6_info *sprt;
	int mpri = 0;

	for (sprt = rt; sprt; sprt = sprt->u.next) {
		struct neighbour *neigh;
		int m = 0;

		if (!oif ||
		    (sprt->rt6i_dev &&
		     sprt->rt6i_dev->ifindex == oif))
			m += 8;

		if (rt6_check_expired(sprt))
			continue;

		if (sprt == rt6_dflt_pointer)
			m += 4;

		if ((neigh = sprt->rt6i_nexthop) != NULL) {
			read_lock_bh(&neigh->lock);
			switch (neigh->nud_state) {
			case NUD_REACHABLE:
				m += 3;
				break;

			case NUD_STALE:
			case NUD_DELAY:
			case NUD_PROBE:
				m += 2;
				break;

			case NUD_NOARP:
			case NUD_PERMANENT:
				m += 1;
				break;

			case NUD_INCOMPLETE:
			default:
				read_unlock_bh(&neigh->lock);
				continue;
			}
			read_unlock_bh(&neigh->lock);
		} else {
			continue;
		}

		if (m > mpri || m >= 12) {
			match = sprt;
			mpri = m;
			if (m >= 12) {
				/* we choose the last default router if it
				 * is in (probably) reachable state.
				 * If route changed, we should do pmtu
				 * discovery. --yoshfuji
				 */
				break;
			}
		}
	}

	spin_lock(&rt6_dflt_lock);
	if (!match) {
		/*
		 *	No default routers are known to be reachable.
		 *	SHOULD round robin
		 */
		if (rt6_dflt_pointer) {
			for (sprt = rt6_dflt_pointer->u.next;
			     sprt; sprt = sprt->u.next) {
				if (sprt->u.dst.obsolete <= 0 &&
				    sprt->u.dst.error == 0 &&
				    !rt6_check_expired(sprt)) {
					match = sprt;
					break;
				}
			}
			for (sprt = rt;
			     !match && sprt;
			     sprt = sprt->u.next) {
				if (sprt->u.dst.obsolete <= 0 &&
				    sprt->u.dst.error == 0 &&
				    !rt6_check_expired(sprt)) {
					match = sprt;
					break;
				}
				if (sprt == rt6_dflt_pointer)
					break;
			}
		}
	}

	if (match) {
		if (rt6_dflt_pointer != match)
			RT6_TRACE("changed default router: %p->%p\n",
				  rt6_dflt_pointer, match);
		rt6_dflt_pointer = match;
	}
	spin_unlock(&rt6_dflt_lock);

	if (!match) {
		/*
		 * Last Resort: if no default routers found, 
		 * use addrconf default route.
		 * We don't record this route.
		 */
		for (sprt = table->root.leaf;
		     sprt; sprt = sprt->u.next) {
			if (!rt6_check_expired(sprt) &&
			    (sprt->rt6i_flags & RTF_DEFAULT) &&
			    (!oif ||
			     (sprt->rt6i_dev &&
			      sprt->rt6i_dev->ifindex == oif))) {
				match = sprt;
				break;
			}
		}
		if (!match) {
			/* no default route.  give up. */
			match = &ip6_null_entry;
		}
	}

	return match;
}

// ori rt6_lookup()
struct rt6_info *rt6_fl_tree_lookup(struct rt6_table *table, 
				    struct flowi *fl, int strict)
{
	struct fib6_node *fn;
	struct rt6_info *rt;

	BUG_TRAP(table != NULL);

	read_lock_bh(&table->lock);
	fn = fib6_lookup(&table->root, &fl->fl6_dst, &fl->fl6_src);
#ifdef CONFIG_IPV6_SUBTREES
	/* unspecfied source address used for lookup */
	while (!(fn->fn_flags & RTN_RTINFO))
		fn = fn->parent;
#endif
	rt = rt6_device_match(fn->leaf, fl->oif, strict);
	dst_hold(&rt->u.dst);
	rt->u.dst.__use++;
	read_unlock_bh(&table->lock);

	rt->u.dst.lastuse = jiffies;
	if (rt->u.dst.error == 0)
		return rt;
	dst_release(&rt->u.dst);
	return NULL;
}

int ip6_ins_rt(struct rt6_info *rt, struct nlmsghdr *nlh,
		void *_rtattr, struct netlink_skb_parms *req) // rt6_tb_ins
{
	int err = -ENOBUFS;

	if (table) {
		write_lock_bh(&table->lock);
		err = fib6_add(&ip6_routing_table, rt, nlh, _rtattr, req);
		write_unlock_bh(&table->lock);
	}
	return err;
}

/* rt6_ins is called with FREE rt6_lock and table->lock.
   It takes new route entry, the addition fails by any reason the
   route is freed. In any case, if caller does not hold it, it may
   be destroyed.
 */
// ori ip6_ins_rt()
int rt6_ins(struct rt6_info *rt, struct nlmsghdr *nlh,
		void *_rtattr)
{
	int err;
	struct rt6_table *table;

	write_lock_bh(&rt6_lock);
	if (unlikely((table = rt6_new_table(rt->rt6i_table)) == NULL)) {
		err = -ENOBUFS;
		goto out;
	}
	err = ip6_ins_rt(rt, NULL, NULL, req); // rt6_tb_ins
out:
	write_unlock_bh(&rt6_lock);

	return err;
}

/* No table->lock! If COW failed, the function returns dead route entry
   with dst->error set to errno value.
 */

static struct rt6_info *rt6_cow(struct rt6_info *ort, struct in6_addr *daddr,
				struct in6_addr *saddr, struct netlink_skb_parms *req)
{
	int err;
	struct rt6_info *rt;

	/*
	 *	Clone the route.
	 */

	rt = ip6_rt_copy(ort);

	if (rt) {
		struct rt6_table *table = rt6_get_table(rt->rt6i_table);

		ipv6_addr_copy(&rt->rt6i_dst.addr, daddr);

		if (!(rt->rt6i_flags&RTF_GATEWAY))
			ipv6_addr_copy(&rt->rt6i_gateway, daddr);

		rt->rt6i_dst.plen = 128;
		rt->rt6i_flags |= RTF_CACHE;
		rt->u.dst.flags |= DST_HOST;

#ifdef CONFIG_IPV6_SUBTREES
		if (saddr) {
			ipv6_addr_copy(&rt->rt6i_src.addr, saddr);
			rt->rt6i_src.plen = 128;
		}
		else
			rt->rt6i_src.plen = ort->rt6i_src.plen;
#endif

		rt->rt6i_nexthop = ndisc_get_neigh(rt->rt6i_dev, &rt->rt6i_gateway);

		dst_hold(&rt->u.dst);

		err = rt6_tb_ins(table, rt, NULL, NULL, req); // ori ip6_ins_rt
		if (err == 0)
			return rt;

		rt->u.dst.error = err;

		return rt;
	}
	dst_hold(&ip6_null_entry.u.dst);
	return &ip6_null_entry;
}

#define BACKTRACK() \
if (rt == &ip6_null_entry && strict) { \
       while ((pn = fn->parent) != NULL) { \
		/* todo: check the RTN_TL_ROOT rule --vjn */ \
		if (FIB6_SUBTREE(pn) != NULL && pn->subtree != fn) { \
			fn = fib6_subtree_lookup(pn, &fl->fl6_src); \
		} else { \
			fn = pn; \
		} \
		if (fn->fn_flags & RTN_TL_ROOT) { \
			dst_hold(&rt->u.dst); \
			goto out; \
		} \
		if (fn->fn_flags & RTN_RTINFO) \
			goto restart; \
	} \
}


struct rt6_info *rt6_fl_tree_input(struct rt6_table *table,
				   struct flowi *fl, int strict) // new ip6_route_input
{
	struct fib6_node *fn, *pn;
	struct rt6_info *rt;
	int attempts = 3;

	BUG_TRAP(table != NULL);

relookup:
	read_lock_bh(&table->lock);

	fn = fib6_lookup(&table->root, &fl->fl6_dst,
			 &fl->fl6_src);

#ifdef CONFIG_IPV6_SUBTREES
	/* unspecfied source address used for lookup */
	while (!(fn->fn_flags & RTN_RTINFO))
	fn = fn->parent;
#endif
restart:
	rt = fn->leaf;

	if ((rt->rt6i_flags & RTF_CACHE)) {
		rt = rt6_device_match(rt, fl->iif, strict);
		BACKTRACK();
		dst_hold(&rt->u.dst);
		goto out;
	}

	rt = rt6_device_match(rt, fl->iif, 0);
	BACKTRACK();

	if (!rt->rt6i_nexthop && !(rt->rt6i_flags & RTF_NONEXTHOP)) {
		struct rt6_info *nrt;
		dst_hold(&rt->u.dst);
		read_unlock_bh(&table->lock);

		nrt = rt6_cow(rt, &fl->fl6_dst,
			      &fl->fl6_src,
			      &NETLINK_CB(skb));

		dst_release(&rt->u.dst);
		rt = nrt;

		if (rt->u.dst.error != -EEXIST || --attempts <= 0)
			goto out2;

		/* Race condition! In the gap, when table->lock was
		   released someone could insert this route.  Relookup.
		*/
		dst_release(&rt->u.dst);
		goto relookup;
	}
	dst_hold(&rt->u.dst);

out:
	read_unlock_bh(&table->lock);
out2:
	rt->u.dst.lastuse = jiffies;
	rt->u.dst.__use++;
	return rt;
}

#ifdef CONFIG_IPV6_SUBTREES
struct rt6_get_saddr_t
{
	struct fib6_walker_t w;
	struct flowi *fl;
	struct in6_addr *saddr;
	int splen;
	int strict;
	struct rt6_info **rt;
};

static int rt6_get_saddr_node(struct fib6_walker_t *w)
{
	struct rt6_get_saddr_t *s = (struct rt6_get_saddr_t*) w;
	struct flowi *fl = s->fl;
	struct rt6_info *rt;

	rt = rt6_device_match(w->leaf, fl->oif, s->strict);

	if (rt != &ip6_null_entry &&
	    rt->rt6i_src.plen > s->splen) {
		struct in6_addr saddr;
		if (!ipv6_get_saddr(&rt->u.dst, &fl->fl6_dst, &saddr)) {
			ipv6_addr_copy(s->saddr, &saddr);
			*s->rt = rt;
			s->splen = rt->rt6i_src.plen;
			if (s->splen == 128)
				w->node = NULL;
		}
	}
	w->leaf = NULL;
	return 0;
}
#endif

static int rt6_get_saddr(struct rt6_info **rt, struct fib6_node *fn,
			 struct flowi *fl, struct in6_addr *saddr, int strict)
{
	int err = -ENETUNREACH;
#ifdef CONFIG_IPV6_SUBTREES
	if (FIB6_SUBTREE(fn)) {
		struct rt6_get_saddr_t s;
		memset(&s, 0, sizeof(s));

		s.w.root = fn->subtree;
		s.w.func = rt6_get_saddr_node;
		s.fl = fl;
		s.saddr = saddr;
		s.strict = strict;
		s.rt = rt;

		fib6_walk(&s.w);

		if (s.splen > 0)
			return 0;
	}
#endif
	if (fn->fn_flags & RTN_RTINFO) {
		*rt = rt6_device_match(fn->leaf, fl->oif, strict);
		if (*rt == &ip6_null_entry)
			return err;
		err = ipv6_get_saddr(&(*rt)->u.dst, &fl->fl6_dst, saddr);
	}
	return err;
}

#ifdef CONFIG_IPV6_SUBTREES
#define RT6_CHK_SRC_PLEN(rt) (rt->rt6i_src.plen == 128)
#else
#define RT6_CHK_SRC_PLEN(rt) 1
#endif
// ori ip6_route_output()
struct rt6_info *rt6_fl_tree_output(struct rt6_table *table, struct flowi *fl, int strict)
{
	struct fib6_node *fn, *pn;
	struct rt6_info *rt = &ip6_null_entry;
	int attempts = 3;
	struct in6_addr saddr;

	BUG_TRAP(table != NULL);

relookup:
	read_lock_bh(&table->lock);

	fn = fib6_lookup(&table->root, &fl->fl6_dst, &fl->fl6_src);

restart:
	if (fn->fn_flags & RTN_RTINFO)
		rt = fn->leaf;

	ipv6_addr_copy(&saddr, &fl->fl6_src);

	if (!ipv6_addr_any(&fl->fl6_src))
		goto saddr_found;

	for(;;) {
		if ((FIB6_SUBTREE(fn) || fn->fn_flags & RTN_RTINFO) &&
		    !rt6_get_saddr(&rt, fn, fl, &saddr, strict)) {
			if (rt != &ip6_null_entry) {
				fn = rt->rt6i_node;
				break;
			}
		}
		if (fn->fn_flags & RTN_TL_ROOT) {
			rt = &ip6_null_entry;
			dst_hold(&rt->u.dst);
			goto out;
		}
		fn = fn->parent;
	}
saddr_found:
	if ((rt->rt6i_flags & RTF_CACHE) && RT6_CHK_SRC_PLEN(rt)) {
		rt = rt6_device_match(rt, fl->oif, strict);
		BACKTRACK();
		dst_hold(&rt->u.dst);
		ipv6_addr_copy(&fl->fl6_src, &saddr);
		goto out;
	}
	if (rt->rt6i_flags & RTF_DEFAULT) {
		if (rt->rt6i_metric >= IP6_RT_PRIO_ADDRCONF)
			rt = rt6_best_dflt(table, rt, fl->oif);
	} else {
		rt = rt6_device_match(rt, fl->oif, strict);
		BACKTRACK();
	}
	ipv6_addr_copy(&fl->fl6_src, &saddr);

	if ((!rt->rt6i_nexthop && !(rt->rt6i_flags & RTF_NONEXTHOP)) ||
	    rt->rt6i_flags & RTF_CACHE) {
		struct rt6_info *nrt;
		dst_hold(&rt->u.dst);
		read_unlock_bh(&table->lock);

		nrt = rt6_cow(rt, &fl->fl6_dst, &fl->fl6_src, NULL);

		dst_release(&rt->u.dst);
		rt = nrt;

		if (rt->u.dst.error != -EEXIST || --attempts <= 0)
			goto out2;

		/* Race condition! In the gap, when table->lock was
		   released someone could insert this route.  Relookup.
		*/
		dst_release(&rt->u.dst);
		goto relookup;
	}
	dst_hold(&rt->u.dst);

out:
	read_unlock_bh(&table->lock);
out2:
	rt->u.dst.lastuse = jiffies;
	rt->u.dst.__use++;
	return rt;
}


/*
 *	Destination cache support functions
 */

static struct dst_entry *ip6_dst_check(struct dst_entry *dst, u32 cookie)
{
	struct rt6_info *rt;

	rt = (struct rt6_info *) dst;

	if (rt && rt->rt6i_node && cookie >= atomic_read(&flow_cache_genid))
		return dst;

	return NULL;
}

static struct dst_entry *ip6_negative_advice(struct dst_entry *dst)
{
	struct rt6_info *rt = (struct rt6_info *) dst;

	if (rt) {
		if (rt->rt6i_flags & RTF_CACHE)
			ip6_del_rt(rt, NULL, NULL, NULL);
		else
			dst_release(dst);
	}
	return NULL;
}

static void ip6_link_failure(struct sk_buff *skb)
{
	struct rt6_info *rt;

	icmpv6_send(skb, ICMPV6_DEST_UNREACH, ICMPV6_ADDR_UNREACH, 0, skb->dev);

	rt = (struct rt6_info *) skb->dst;
	if (rt) {
		if (rt->rt6i_flags&RTF_CACHE) {
			dst_set_expires(&rt->u.dst, 0);
			rt->rt6i_flags |= RTF_EXPIRES;
		}
	}
}

static void ip6_rt_update_pmtu(struct dst_entry *dst, u32 mtu)
{
	struct rt6_info *rt6 = (struct rt6_info*)dst;

	if (mtu < dst_mtu(dst) && rt6->rt6i_dst.plen == 128) {
		rt6->rt6i_flags |= RTF_MODIFIED;
		if (mtu < IPV6_MIN_MTU) {
			mtu = IPV6_MIN_MTU;
			dst->metrics[RTAX_FEATURES-1] |= RTAX_FEATURE_ALLFRAG;
		}
		dst->metrics[RTAX_MTU-1] = mtu;
	}
}

/* Protected by ndisc_lock.  */
static struct dst_entry *ndisc_dst_gc_list;
static int ipv6_get_mtu(struct net_device *dev);

static inline unsigned int ipv6_advmss(unsigned int mtu)
{
	mtu -= sizeof(struct ipv6hdr) + sizeof(struct tcphdr);

	if (mtu < ip6_rt_min_advmss)
		mtu = ip6_rt_min_advmss;

	/*
	 * Maximal non-jumbo IPv6 payload is IPV6_MAXPLEN and 
	 * corresponding MSS is IPV6_MAXPLEN - tcp_header_size. 
	 * IPV6_MAXPLEN is also valid and means: "any MSS, 
	 * rely only on pmtu discovery"
	 */
	if (mtu > IPV6_MAXPLEN - sizeof(struct tcphdr))
		mtu = IPV6_MAXPLEN;
	return mtu;
}

struct dst_entry *ndisc_dst_alloc(struct net_device *dev, 
				  struct neighbour *neigh,
				  struct in6_addr *addr,
				  int (*output)(struct sk_buff *))
{
	struct rt6_info *rt;
	struct inet6_dev *idev = in6_dev_get(dev);

	if (unlikely(idev == NULL))
		return NULL;

	rt = ip6_dst_alloc();
	if (unlikely(rt == NULL)) {
		in6_dev_put(idev);
		goto out;
	}

	dev_hold(dev);
	if (neigh)
		neigh_hold(neigh);
	else
		neigh = ndisc_get_neigh(dev, addr);

	rt->rt6i_dev	  = dev;
	rt->rt6i_idev     = idev;
	rt->rt6i_nexthop  = neigh;
	atomic_set(&rt->u.dst.__refcnt, 1);
	rt->u.dst.metrics[RTAX_HOPLIMIT-1] = 255;
	rt->u.dst.metrics[RTAX_MTU-1] = ipv6_get_mtu(rt->rt6i_dev);
	rt->u.dst.metrics[RTAX_ADVMSS-1] = ipv6_advmss(dst_mtu(&rt->u.dst));
	rt->u.dst.output  = output;
	rt->u.dst.obsolete = -1;

#if 0	/* there's no chance to use these for ndisc */
	rt->u.dst.flags   = ipv6_addr_type(addr) & IPV6_ADDR_UNICAST 
				? DST_HOST 
				: 0;
	ipv6_addr_copy(&rt->rt6i_dst.addr, addr);
	rt->rt6i_dst.plen = 128;
#endif

	spin_lock_bh(&ndisc_lock);
	rt->u.dst.next = ndisc_dst_gc_list;
	ndisc_dst_gc_list = &rt->u.dst;
	spin_unlock_bh(&ndisc_lock);

	fib6_force_start_gc();

out:
	return (struct dst_entry *)rt;
}

int ndisc_dst_gc(int *more)
{
	struct dst_entry *dst, *next, **pprev;
	int freed;

	next = NULL;
	freed = 0;
	spin_lock_bh(&ndisc_lock);
	pprev = &ndisc_dst_gc_list;
	while ((dst = *pprev) != NULL) {
		if (!atomic_read(&dst->__refcnt)) {
			*pprev = dst->next;
			dst_free(dst);
			freed++;
		} else {
			pprev = &dst->next;
			(*more)++;
		}
	}
	spin_unlock_bh(&ndisc_lock);

	return freed;
}

static int ip6_dst_gc(void)
{
	static unsigned expire = 30*HZ;
	static unsigned long last_gc;
	unsigned long now = jiffies;

	if (time_after(last_gc + ip6_rt_gc_min_interval, now) &&
	    atomic_read(&ip6_dst_ops.entries) <= ip6_rt_max_size)
		goto out;

	expire++;
	fib6_run_gc(expire);
	last_gc = now;
	if (atomic_read(&ip6_dst_ops.entries) < ip6_dst_ops.gc_thresh)
		expire = ip6_rt_gc_timeout>>1;

out:
	expire -= expire>>ip6_rt_gc_elasticity;
	return (atomic_read(&ip6_dst_ops.entries) > ip6_rt_max_size);
}

/* Clean host part of a prefix. Not necessary in radix tree,
   but results in cleaner routing tables.

   Remove it only when all the things will work!
 */

static int ipv6_get_mtu(struct net_device *dev)
{
	int mtu = IPV6_MIN_MTU;
	struct inet6_dev *idev;

	idev = in6_dev_get(dev);
	if (idev) {
		mtu = idev->cnf.mtu6;
		in6_dev_put(idev);
	}
	return mtu;
}

int ipv6_get_hoplimit(struct net_device *dev)
{
	int hoplimit = ipv6_devconf.hop_limit;
	struct inet6_dev *idev;

	idev = in6_dev_get(dev);
	if (idev) {
		hoplimit = idev->cnf.hop_limit;
		in6_dev_put(idev);
	}
	return hoplimit;
}

/*
 *
 */

static inline int rt6_get_table_id(struct nlmsghdr *nlh)
{
	struct rtmsg *r;

	if (nlh && (r = NLMSG_DATA(nlh)) && r->rtm_table)
		return r->rtm_table;

	return RT6_TABLE_MAIN;
}

// ori ip6_route_add
int ip6_tb_route_add(int table_id, struct in6_rtmsg *rtmsg, struct nlmsghdr *nlh, 
		void *_rtattr, struct netlink_skb_parms *req)
{
	int err;
	struct rtmsg *r;
	struct rtattr **rta;
	struct rt6_info *rt = NULL;
	struct net_device *dev = NULL;
	struct inet6_dev *idev = NULL;
	int addr_type;

	rta = (struct rtattr **) _rtattr;

	if (rtmsg->rtmsg_dst_len > 128 || rtmsg->rtmsg_src_len > 128)
		return -EINVAL;
#ifndef CONFIG_IPV6_SUBTREES
	if (rtmsg->rtmsg_src_len)
		return -EINVAL;
#endif
	if (rtmsg->rtmsg_ifindex) {
		err = -ENODEV;
		dev = dev_get_by_index(rtmsg->rtmsg_ifindex);
		if (!dev)
			goto out;
		idev = in6_dev_get(dev);
		if (!idev)
			goto out;
	}

	if (rtmsg->rtmsg_metric == 0)
		rtmsg->rtmsg_metric = IP6_RT_PRIO_USER;

	rt = ip6_dst_alloc();

	if (rt == NULL) {
		err = -ENOMEM;
		goto out;
	}

	rt->u.dst.obsolete = -1;
	rt->rt6i_expires = clock_t_to_jiffies(rtmsg->rtmsg_info);
	if (nlh && (r = NLMSG_DATA(nlh))) {
		rt->rt6i_protocol = r->rtm_protocol;
	} else {
		rt->rt6i_protocol = RTPROT_BOOT;
	}
	if (table_id != RT6_TABLE_UNSPEC)
		rt->rt6i_table = table_id;
	else
		rt->rt6i_table = RT6_TABLE_MAIN;

	addr_type = ipv6_addr_type(&rtmsg->rtmsg_dst);

	if (addr_type & IPV6_ADDR_MULTICAST)
		rt->u.dst.input = ip6_mc_input;
	else
		rt->u.dst.input = ip6_forward;

	rt->u.dst.output = ip6_output;

	ipv6_addr_prefix(&rt->rt6i_dst.addr, 
			 &rtmsg->rtmsg_dst, rtmsg->rtmsg_dst_len);
	rt->rt6i_dst.plen = rtmsg->rtmsg_dst_len;
	if (rt->rt6i_dst.plen == 128)
	       rt->u.dst.flags = DST_HOST;

#ifdef CONFIG_IPV6_SUBTREES
	ipv6_addr_prefix(&rt->rt6i_src.addr, 
			 &rtmsg->rtmsg_src, rtmsg->rtmsg_src_len);
	rt->rt6i_src.plen = rtmsg->rtmsg_src_len;
#endif

	rt->rt6i_metric = rtmsg->rtmsg_metric;

	/* We cannot add true routes via loopback here,
	   they would result in kernel looping; promote them to reject routes
	 */
	if ((rtmsg->rtmsg_flags&RTF_REJECT) ||
	    (dev && (dev->flags&IFF_LOOPBACK) && !(addr_type&IPV6_ADDR_LOOPBACK))) {
		/* hold loopback dev/idev if we haven't done so. */
		if (dev != &loopback_dev) {
			if (dev) {
				dev_put(dev);
				in6_dev_put(idev);
			}
			dev = &loopback_dev;
			dev_hold(dev);
			idev = in6_dev_get(dev);
			if (!idev) {
				err = -ENODEV;
				goto out;
			}
		}
		rt->u.dst.output = ip6_pkt_discard_out;
		rt->u.dst.input = ip6_pkt_discard;
		rt->u.dst.error = -ENETUNREACH;
		rt->rt6i_flags = RTF_REJECT|RTF_NONEXTHOP;
		goto install_route;
	}

	if (rtmsg->rtmsg_flags & RTF_GATEWAY) {
		struct in6_addr *gw_addr;
		int gwa_type;

		gw_addr = &rtmsg->rtmsg_gateway;
		ipv6_addr_copy(&rt->rt6i_gateway, &rtmsg->rtmsg_gateway);
		gwa_type = ipv6_addr_type(gw_addr);

		if (gwa_type != (IPV6_ADDR_LINKLOCAL|IPV6_ADDR_UNICAST)) {
			struct rt6_info *grt;

			/* IPv6 strictly inhibits using not link-local
			   addresses as nexthop address.
			   Otherwise, router will not able to send redirects.
			   It is very good, but in some (rare!) circumstances
			   (SIT, PtP, NBMA NOARP links) it is handy to allow
			   some exceptions. --ANK
			 */
			err = -EINVAL;
			if (!(gwa_type&IPV6_ADDR_UNICAST))
				goto out;

			grt = rt6_lookup(gw_addr, &rtmsg->rtmsg_src, rtmsg->rtmsg_ifindex, 1);

			err = -EHOSTUNREACH;
			if (grt == NULL)
				goto out;
			if (dev) {
				if (dev != grt->rt6i_dev) {
					dst_release(&grt->u.dst);
					goto out;
				}
			} else {
				dev = grt->rt6i_dev;
				idev = grt->rt6i_idev;
				dev_hold(dev);
				in6_dev_hold(grt->rt6i_idev);
			}
			if (!(grt->rt6i_flags&RTF_GATEWAY))
				err = 0;
			dst_release(&grt->u.dst);

			if (err)
				goto out;
		}
		err = -EINVAL;
		if (dev == NULL || (dev->flags&IFF_LOOPBACK))
			goto out;
	}

	err = -ENODEV;
	if (dev == NULL)
		goto out;

	if (rtmsg->rtmsg_flags & (RTF_GATEWAY|RTF_NONEXTHOP)) {
		rt->rt6i_nexthop = __neigh_lookup_errno(&nd_tbl, &rt->rt6i_gateway, dev);
		if (IS_ERR(rt->rt6i_nexthop)) {
			err = PTR_ERR(rt->rt6i_nexthop);
			rt->rt6i_nexthop = NULL;
			goto out;
		}
	}

	rt->rt6i_flags = rtmsg->rtmsg_flags;

install_route:
	if (rta && rta[RTA_METRICS-1]) {
		int attrlen = RTA_PAYLOAD(rta[RTA_METRICS-1]);
		struct rtattr *attr = RTA_DATA(rta[RTA_METRICS-1]);

		while (RTA_OK(attr, attrlen)) {
			unsigned flavor = attr->rta_type;
			if (flavor) {
				if (flavor > RTAX_MAX) {
					err = -EINVAL;
					goto out;
				}
				rt->u.dst.metrics[flavor-1] =
					*(u32 *)RTA_DATA(attr);
			}
			attr = RTA_NEXT(attr, attrlen);
		}
	}

	if (rt->u.dst.metrics[RTAX_HOPLIMIT-1] == 0)
		rt->u.dst.metrics[RTAX_HOPLIMIT-1] = -1;
	if (!rt->u.dst.metrics[RTAX_MTU-1])
		rt->u.dst.metrics[RTAX_MTU-1] = ipv6_get_mtu(dev);
	if (!rt->u.dst.metrics[RTAX_ADVMSS-1])
		rt->u.dst.metrics[RTAX_ADVMSS-1] = ipv6_advmss(dst_mtu(&rt->u.dst));
	rt->u.dst.dev = dev;
	rt->rt6i_idev = idev;
	return ip6_ins_rt(rt, nlh, _rtattr, req);

out:
	if (dev)
		dev_put(dev);
	if (idev)
		in6_dev_put(idev);
	if (rt)
		dst_free((struct dst_entry *) rt);
	return err;
}

// ori ip6_del_rt()
int ip6_tb_del_rt(struct rt6_table *table, 
		struct rt6_info *rt, struct nlmsghdr *nlh, void *_rtattr, struct netlink_skb_parms *req)
{
	int err;

	write_lock_bh(&table->lock);

	rt6_reset_dflt_pointer(NULL);

	err = fib6_del(rt, nlh, _rtattr, req);
	dst_release(&rt->u.dst);

	write_unlock_bh(&table->lock);

	return err;
}

int ip6_del_rt(struct rt6_info *rt, struct nlmsghdr *nlh, void *_rtattr, struct netlink_skb_parms *req)
{
	int err;
	struct rt6_table *table;
	read_lock_bh(&rt6_lock);
	table = rt6_get_table(rt->rt6i_table);
	BUG_TRAP(table != NULL);
	err = ip6_tb_del_rt(table, rt, nlh, _rtattr);
	read_unlock_bh(&rt6_lock);

	return err;
}

// old ip6_tb_route_del
static int ip6_tb_route_del(int table_id,
		struct in6_rtmsg *rtmsg, struct nlmsghdr *nlh, void *_rtattr, struct netlink_skb_parms *req)
{
	struct rt6_table *table;
	struct fib6_node *fn;
	struct rt6_info *rt;
	int err = -ESRCH;

	read_lock_bh(&rt6_lock);
	table = rt6_get_table(table_id);
if (likely(table)) {
	read_lock(&table->lock);
	fn = fib6_locate(&table->root,
			 &rtmsg->rtmsg_dst, rtmsg->rtmsg_dst_len,
			 &rtmsg->rtmsg_src, rtmsg->rtmsg_src_len);

	if (fn) {
		for (rt = fn->leaf; rt; rt = rt->u.next) {
			if (rtmsg->rtmsg_ifindex &&
			    (rt->rt6i_dev == NULL ||
			     rt->rt6i_dev->ifindex != rtmsg->rtmsg_ifindex))
				continue;
			if (rtmsg->rtmsg_flags&RTF_GATEWAY &&
			    ipv6_addr_cmp(&rtmsg->rtmsg_gateway, &rt->rt6i_gateway))
				continue;
			if (rtmsg->rtmsg_metric &&
			    rtmsg->rtmsg_metric != rt->rt6i_metric)
				continue;
			dst_hold(&rt->u.dst);
			read_unlock(&table->lock);

			return ip6_del_rt(rt, nlh, _rtattr, req); // mvl ip6_del_rt
			goto out;
		}
	}
		read_unlock(&table->lock);
}
out:
	read_unlock_bh(&rt6_lock);

	return err;
}

static inline void rt6_tb_redirect(struct rt6_table *table,
				   struct rt6_info *rt,
				   struct in6_addr *dest,
				   struct in6_addr *saddr,
				   struct neighbour *neigh, int on_link)
{
	struct rt6_info *nrt;

	if (neigh->dev != rt->rt6i_dev)
		return;

	/* Redirect received -> path was valid.
	   Look, redirects are sent only in response to data packets,
	   so that this nexthop apparently is reachable. --ANK
	*/
	dst_confirm(&rt->u.dst);

	/* Duplicate redirect: silently ignore. */
	if (neigh == rt->u.dst.neighbour)
		return;

	/* Current route is on-link; redirect is always invalid.
	   
	   Seems, previous statement is not true. It could
	   be node, which looks for us as on-link (f.e. proxy ndisc)
	   But then router serving it might decide, that we should
	   know truth 8)8) --ANK (980726).
	 */
	if (!(rt->rt6i_flags&RTF_GATEWAY) ||
	    !ipv6_addr_equal(saddr, &rt->rt6i_gateway))
		return;

/*#ifdef 0
	neigh_update(neigh, lladdr, NUD_STALE, 
		     NEIGH_UPDATE_F_WEAK_OVERRIDE|
		     NEIGH_UPDATE_F_OVERRIDE|
		     (on_link ? 0 : (NEIGH_UPDATE_F_OVERRIDE_ISROUTER|
				     NEIGH_UPDATE_F_ISROUTER))
		     );
#endif */

	nrt = ip6_rt_copy(rt);
	if (nrt == NULL)
		return;

	nrt->rt6i_flags = RTF_GATEWAY|RTF_UP|RTF_DYNAMIC|RTF_CACHE;
	if (on_link)
		nrt->rt6i_flags &= ~RTF_GATEWAY;

	ipv6_addr_copy(&nrt->rt6i_dst.addr, dest);
	nrt->rt6i_dst.plen = 128;
	nrt->u.dst.flags |= DST_HOST;

	ipv6_addr_copy(&nrt->rt6i_gateway, (struct in6_addr*)neigh->primary_key);
	nrt->rt6i_nexthop = neigh_clone(neigh);
	/* Reset pmtu, it may be better */
	nrt->u.dst.metrics[RTAX_MTU-1] = ipv6_get_mtu(neigh->dev);
	nrt->u.dst.metrics[RTAX_ADVMSS-1] = ipv6_advmss(dst_mtu(&nrt->u.dst));

	if (rt6_tb_ins(nrt, NULL, NULL, NULL)) // pld ip6_ins_rt()
		return;

	if (rt->rt6i_flags&RTF_CACHE) {
		dst_hold(&rt->u.dst);
		ip6_tb_del_rt(table, rt, NULL, NULL);
		return;
	}
}

/*
 *	Handle redirects
 */
void rt6_redirect(struct in6_addr *dest, struct in6_addr *saddr,
		  struct neighbour *neigh, int on_link)
{
	int i;

	/* Locate old routes to this destination. */

	read_lock(&rt6_lock);
	for (i = RT6_TABLE_MIN; i <= RT6_TABLE_MAX; i++) {
		struct rt6_info *rtbase, *rt;
		struct rt6_table *table = rt6_get_table(i);

		if (!table)
			continue;

		rtbase = rt6_tb_lookup(table, dest, saddr, 
				       neigh->dev->ifindex, 1);
		rt = rtbase;
		while (rt) {
			struct rt6_info *nrt;
			read_lock(&table->lock);
			if ((nrt = rt->u.next))
				dst_hold(&nrt->u.dst);
			read_unlock(&table->lock);
			
			rt6_tb_redirect(table, rt, dest, saddr, 
					neigh, on_link);
			
			dst_release(&rt->u.dst);
			rt = nrt;
		}
	}
	read_unlock(&rt6_lock);
}

// tolko mvl otdelnaj funkcia
// old rt6_pmtu_discovery()
static inline void rt6_tb_pmtu_discovery(struct rt6_table *table, struct rt6_info *rt, 
			struct in6_addr *daddr, struct in6_addr *saddr,
			struct net_device *dev, u32 pmtu)
{
	struct rt6_info *nrt;
	int allfrag = 0;
	
	if (pmtu >= dst_mtu(&rt->u.dst))
		return;
		
	if (pmtu < IPV6_MIN_MTU) {
		/*
		 * According to RFC2460, PMTU is set to the IPv6 Minimum Link 
		 * MTU (1280) and a fragment header should always be included
		 * after a node receiving Too Big message reporting PMTU is
		 * less than the IPv6 Minimum Link MTU.
		 */
		pmtu = IPV6_MIN_MTU;
		allfrag = 1;
	}

	/* New mtu received -> path was valid.
	   They are sent only in response to data packets,
	   so that this nexthop apparently is reachable. --ANK
	 */
	dst_confirm(&rt->u.dst);

	/* Host route. If it is static, it would be better
	   not to override it, but add new one, so that
	   when cache entry will expire old pmtu
	   would return automatically.
	 */
	if (rt->rt6i_flags & RTF_CACHE) {
		rt->u.dst.metrics[RTAX_MTU-1] = pmtu;
		if (allfrag)
			rt->u.dst.metrics[RTAX_FEATURES-1] |= RTAX_FEATURE_ALLFRAG;
		dst_set_expires(&rt->u.dst, ip6_rt_mtu_expires);
		rt->rt6i_flags |= RTF_MODIFIED|RTF_EXPIRES;
		return;
	}

	/* Network route.
	   Two cases are possible:
	   1. It is connected route. Action: COW
	   2. It is gatewayed route or NONEXTHOP route. Action: clone it.
	 */
	if (!rt->rt6i_nexthop && !(rt->rt6i_flags & RTF_NONEXTHOP)) {
		nrt = rt6_cow(rt, daddr, saddr, NULL);
		if (!nrt->u.dst.error) {
			nrt->u.dst.metrics[RTAX_MTU-1] = pmtu;
			if (allfrag)
				nrt->u.dst.metrics[RTAX_FEATURES-1] |= RTAX_FEATURE_ALLFRAG;
			/* According to RFC 1981, detecting PMTU increase shouldn't
			   happened within 5 mins, the recommended timer is 10 mins.
			   Here this route expiration time is set to ip6_rt_mtu_expires
			   which is 10 mins. After 10 mins the decreased pmtu is expired
			   and detecting PMTU increase will be automatically happened.
			 */
			dst_set_expires(&nrt->u.dst, ip6_rt_mtu_expires);
			nrt->rt6i_flags |= RTF_DYNAMIC|RTF_EXPIRES;
		}
		dst_release(&nrt->u.dst);
	} else {
		nrt = ip6_rt_copy(rt);
		if (nrt == NULL)
			return;
#ifdef CONFIG_IPV6_SUBTREES
		nrt->rt6i_src.plen = rt->rt6i_src.plen;
#endif
		ipv6_addr_copy(&nrt->rt6i_dst.addr, daddr);
		nrt->rt6i_dst.plen = 128;
		nrt->u.dst.flags |= DST_HOST;
		nrt->rt6i_nexthop = neigh_clone(rt->rt6i_nexthop);
		dst_set_expires(&nrt->u.dst, ip6_rt_mtu_expires);
		nrt->rt6i_flags |= RTF_DYNAMIC|RTF_CACHE|RTF_EXPIRES;
		nrt->u.dst.metrics[RTAX_MTU-1] = pmtu;
		if (allfrag)
			nrt->u.dst.metrics[RTAX_FEATURES-1] |= RTAX_FEATURE_ALLFRAG;
		ip6_ins_rt(nrt, NULL, NULL, NULL);
	}
}

/*
 *	Handle ICMP "packet too big" messages
 *	i.e. Path MTU discovery
 */

void rt6_pmtu_discovery(struct in6_addr *daddr, struct in6_addr *saddr,
			struct net_device *dev, u32 pmtu)
{
	int allfrag = 0;

	read_lock(&rt6_lock);

	int i;
	for (i = RT6_TABLE_MIN; i <= RT6_TABLE_MAX; i++) {
		struct rt6_info *rtbase, *rt;
		struct rt6_table *table = rt6_get_table(i);

		if (!table)
			continue;

		rtbase = rt6_tb_lookup(table, daddr, saddr, dev->ifindex, 1);
		rt = rtbase;
		while (rt) {
			struct rt6_info *nrt;
			read_lock(&table->lock);
			if ((nrt = rt->u.next))
				dst_hold(&nrt->u.dst);
			read_unlock(&table->lock);
			
			rt6_tb_pmtu_discovery(table, rt, daddr, saddr,
					      dev, pmtu); // tolko mvl otdelnaj funkcia
			
			dst_release(&rt->u.dst);
			rt = nrt;
		}
	}
	read_unlock(&rt6_lock);
}

/*
 *	Misc support functions
 */

static struct rt6_info * ip6_rt_copy(struct rt6_info *ort)
{
	struct rt6_info *rt = ip6_dst_alloc();

	if (rt) {
		rt->u.dst.input = ort->u.dst.input;
		rt->u.dst.output = ort->u.dst.output;
		rt->u.dst.obsolete = -1;

		memcpy(rt->u.dst.metrics, ort->u.dst.metrics, RTAX_MAX*sizeof(u32));
		rt->u.dst.dev = ort->u.dst.dev;
		if (rt->u.dst.dev)
			dev_hold(rt->u.dst.dev);
		rt->rt6i_idev = ort->rt6i_idev;
		if (rt->rt6i_idev)
			in6_dev_hold(rt->rt6i_idev);
		rt->u.dst.lastuse = jiffies;
		rt->rt6i_expires = 0;

		ipv6_addr_copy(&rt->rt6i_gateway, &ort->rt6i_gateway);
		rt->rt6i_flags = ort->rt6i_flags & ~RTF_EXPIRES;
		rt->rt6i_metric = 0;

		memcpy(&rt->rt6i_dst, &ort->rt6i_dst, sizeof(struct rt6key));
#ifdef CONFIG_IPV6_SUBTREES
		memcpy(&rt->rt6i_src, &ort->rt6i_src, sizeof(struct rt6key));
#endif
		rt->rt6i_table = ort->rt6i_table;
	}
	return rt;
}

struct rt6_info *rt6_get_dflt_router(struct in6_addr *addr, struct net_device *dev)
{	
	struct rt6_info *rt;
	struct rt6_table *table;
	struct fib6_node *fn;

	read_lock_bh(&rt6_lock);
	table = rt6_get_table(RT6_TABLE_MAIN);

	fn = &table->root;

	read_lock(&table->lock);
	for (rt = fn->leaf; rt; rt=rt->u.next) {
		if (dev == rt->rt6i_dev &&
		    ipv6_addr_equal(&rt->rt6i_gateway, addr))
			break;
	}
	if (rt)
		dst_hold(&rt->u.dst);
	read_unlock(&table->lock);

	read_unlock_bh(&rt6_lock);
	return rt;
}

/* Fix ME - add back the router preferences */
struct rt6_info *rt6_add_dflt_router(struct in6_addr *gwaddr,
				     struct net_device *dev,
				     int pref)
{
	struct in6_rtmsg rtmsg;

	memset(&rtmsg, 0, sizeof(struct in6_rtmsg));
	rtmsg.rtmsg_type = RTMSG_NEWROUTE;
	ipv6_addr_copy(&rtmsg.rtmsg_gateway, gwaddr);
	rtmsg.rtmsg_metric = 1024;
	rtmsg.rtmsg_flags = RTF_GATEWAY | RTF_ADDRCONF | RTF_DEFAULT | RTF_UP | RTF_EXPIRES;

	rtmsg.rtmsg_ifindex = dev->ifindex;

	ip6_route_add(&rtmsg, NULL, NULL, NULL);
	return rt6_get_dflt_router(gwaddr, dev);
}

void rt6_purge_dflt_routers(void)
{
	struct rt6_table *table;
	struct rt6_info *rt;

	read_lock_bh(&rt6_lock);
	table = rt6_get_table(RT6_TABLE_MAIN);
restart:
	read_lock(&table->lock);
	for (rt = table->root.leaf; rt; rt = rt->u.next) {
		if (rt->rt6i_flags & (RTF_DEFAULT | RTF_ADDRCONF)) {
			dst_hold(&rt->u.dst);

			rt6_reset_dflt_pointer(NULL);

			read_unlock(&table->lock);

			ip6_tb_del_rt(table, rt, NULL, NULL);

			goto restart;
		}
	}
	read_unlock(&table->lock);
	read_unlock_bh(&rt6_lock);
}

int ipv6_route_ioctl(unsigned int cmd, void __user *arg)
{
	struct in6_rtmsg rtmsg;
	int err;

	switch(cmd) {
	case SIOCADDRT:		/* Add a route */
	case SIOCDELRT:		/* Delete a route */
		if (!capable(CAP_NET_ADMIN))
			return -EPERM;
		err = copy_from_user(&rtmsg, arg,
				     sizeof(struct in6_rtmsg));
		if (err)
			return -EFAULT;
			
		rtnl_lock();
		switch (cmd) {
		case SIOCADDRT:
			err = ip6_tb_route_add(0, &rtmsg, NULL, NULL, NULL);
			break;
		case SIOCDELRT:
			err = ip6_tb_route_del(0, &rtmsg, NULL, NULL, NULL);
			break;
		default:
			err = -EINVAL;
		}
		rtnl_unlock();

		return err;
	};

	return -EINVAL;
}

/*
 *	Drop the packet on the floor
 */

int ip6_pkt_discard(struct sk_buff *skb)
{

	if (skb->dst->error == -ENETUNREACH) {
		IP6_INC_STATS(IPSTATS_MIB_OUTNOROUTES);
		icmpv6_send(skb, ICMPV6_DEST_UNREACH, ICMPV6_NOROUTE, 0, skb->dev);
	} else {
		IP6_INC_STATS(IPSTATS_MIB_OUTDISCARDS);
		if (skb->dst->error == -EACCES)
			icmpv6_send(skb, ICMPV6_DEST_UNREACH, 
				    ICMPV6_ADM_PROHIBITED, 0, skb->dev);
	}
	kfree_skb(skb);
	return 0;
}

int ip6_pkt_discard_out(struct sk_buff *skb)
{
	skb->dev = skb->dst->dev;
	return ip6_pkt_discard(skb);
}

/*
 *	Allocate a dst for local (unicast / anycast) address.
 */

struct rt6_info *addrconf_dst_alloc(const struct in6_addr *addr,
				    int anycast)
{
	struct rt6_info *rt = ip6_dst_alloc();

	if (rt == NULL)
		return ERR_PTR(-ENOMEM);

	dev_hold(&loopback_dev);

	rt->u.dst.flags = DST_HOST;
	rt->u.dst.input = ip6_input;
	rt->u.dst.output = ip6_output;
	rt->rt6i_dev = &loopback_dev;
	rt->rt6i_idev = in6_dev_get(&loopback_dev);
	rt->u.dst.metrics[RTAX_MTU-1] = ipv6_get_mtu(rt->rt6i_dev);
	rt->u.dst.metrics[RTAX_ADVMSS-1] = ipv6_advmss(dst_mtu(&rt->u.dst));
	rt->u.dst.metrics[RTAX_HOPLIMIT-1] = -1;
	rt->u.dst.obsolete = -1;

	rt->rt6i_flags = RTF_UP | RTF_NONEXTHOP;
	if (anycast)
		rt->rt6i_flags |= RTF_ANYCAST;
	else
		rt->rt6i_flags |= RTF_LOCAL;
	rt->rt6i_nexthop = ndisc_get_neigh(rt->rt6i_dev, &rt->rt6i_gateway);
	if (rt->rt6i_nexthop == NULL) {
		dst_free((struct dst_entry *) rt);
		return ERR_PTR(-ENOMEM);
	}

	ipv6_addr_copy(&rt->rt6i_dst.addr, addr);
	rt->rt6i_dst.plen = 128;
	rt->rt6i_table = RT6_TABLE_LOCAL;

	atomic_set(&rt->u.dst.__refcnt, 1);

	return rt;
}

static int fib6_ifdown(struct rt6_info *rt, void *arg)
{
	if (((void*)rt->rt6i_dev == arg || arg == NULL) &&
	    rt != &ip6_null_entry &&
		!(rt->rt6i_flags & (RTF_LOCAL|RTF_ANYCAST))) {
		RT6_TRACE("deleted by ifdown %p\n", rt);
		return -1;
	}
	return 0;
}

void rt6_ifdown(struct net_device *dev)
{
	int i;
	read_lock_bh(&rt6_lock);
	for (i = RT6_TABLE_MIN; i <= RT6_TABLE_MAX; i++) {
		struct rt6_table *table = rt6_get_table(i);
		if (table) {
			write_lock(&table->lock);
			fib6_clean_tree(&table->root, fib6_ifdown, 0, dev);
			write_unlock(&table->lock);
		}
	}
	read_unlock_bh(&rt6_lock);
}

struct rt6_mtu_change_arg
{
	struct net_device *dev;
	unsigned mtu;
};

static int rt6_mtu_change_route(struct rt6_info *rt, void *p_arg)
{
	struct rt6_mtu_change_arg *arg = (struct rt6_mtu_change_arg *) p_arg;
	struct inet6_dev *idev;

	/* In IPv6 pmtu discovery is not optional,
	   so that RTAX_MTU lock cannot disable it.
	   We still use this lock to block changes
	   caused by addrconf/ndisc.
	*/

	idev = __in6_dev_get(arg->dev);
	if (idev == NULL)
		return 0;

	/* For administrative MTU increase, there is no way to discover
	   IPv6 PMTU increase, so PMTU increase should be updated here.
	   Since RFC 1981 doesn't include administrative MTU increase
	   update PMTU increase is a MUST. (i.e. jumbo frame)
	 */
	/*
	   If new MTU is less than route PMTU, this new MTU will be the
	   lowest MTU in the path, update the route PMTU to reflect PMTU
	   decreases; if new MTU is greater than route PMTU, and the
	   old MTU is the lowest MTU in the path, update the route PMTU
	   to reflect the increase. In this case if the other nodes' MTU
	   also have the lowest MTU, TOO BIG MESSAGE will be lead to
	   PMTU discouvery.
	 */
	if (rt->rt6i_dev == arg->dev &&
	    !dst_metric_locked(&rt->u.dst, RTAX_MTU) &&
            (dst_mtu(&rt->u.dst) > arg->mtu ||
             (dst_mtu(&rt->u.dst) < arg->mtu &&
	      dst_mtu(&rt->u.dst) == idev->cnf.mtu6)))
		rt->u.dst.metrics[RTAX_MTU-1] = arg->mtu;
	rt->u.dst.metrics[RTAX_ADVMSS-1] = ipv6_advmss(arg->mtu);
	return 0;
}

void rt6_mtu_change(struct net_device *dev, unsigned mtu)
{
	struct rt6_mtu_change_arg arg;
	int i;

	arg.dev = dev;
	arg.mtu = mtu;
	read_lock_bh(&rt6_lock);
	for (i = RT6_TABLE_MIN; i <= RT6_TABLE_MAX; i++) {
		struct rt6_table *table = rt6_get_table(i);
		if (table) {
			read_lock(&table->lock);
			fib6_clean_tree(&table->root, 
					rt6_mtu_change_route, 0, &arg);
			read_unlock(&table->lock);
		}
	}
	read_unlock_bh(&rt6_lock);
}

static int inet6_rtm_to_rtmsg(struct rtmsg *r, struct rtattr **rta,
			      struct in6_rtmsg *rtmsg)
{
	memset(rtmsg, 0, sizeof(*rtmsg));

	rtmsg->rtmsg_dst_len = r->rtm_dst_len;
	rtmsg->rtmsg_src_len = r->rtm_src_len;
	rtmsg->rtmsg_flags = RTF_UP;
	if (r->rtm_type == RTN_UNREACHABLE)
		rtmsg->rtmsg_flags |= RTF_REJECT;

	if (rta[RTA_GATEWAY-1]) {
		if (rta[RTA_GATEWAY-1]->rta_len != RTA_LENGTH(16))
			return -EINVAL;
		memcpy(&rtmsg->rtmsg_gateway, RTA_DATA(rta[RTA_GATEWAY-1]), 16);
		rtmsg->rtmsg_flags |= RTF_GATEWAY;
	}
	if (rta[RTA_DST-1]) {
		if (RTA_PAYLOAD(rta[RTA_DST-1]) < ((r->rtm_dst_len+7)>>3))
			return -EINVAL;
		memcpy(&rtmsg->rtmsg_dst, RTA_DATA(rta[RTA_DST-1]), ((r->rtm_dst_len+7)>>3));
	}
	if (rta[RTA_SRC-1]) {
		if (RTA_PAYLOAD(rta[RTA_SRC-1]) < ((r->rtm_src_len+7)>>3))
			return -EINVAL;
		memcpy(&rtmsg->rtmsg_src, RTA_DATA(rta[RTA_SRC-1]), ((r->rtm_src_len+7)>>3));
	}
	if (rta[RTA_OIF-1]) {
		if (rta[RTA_OIF-1]->rta_len != RTA_LENGTH(sizeof(int)))
			return -EINVAL;
		memcpy(&rtmsg->rtmsg_ifindex, RTA_DATA(rta[RTA_OIF-1]), sizeof(int));
	}
	if (rta[RTA_PRIORITY-1]) {
		if (rta[RTA_PRIORITY-1]->rta_len != RTA_LENGTH(4))
			return -EINVAL;
		memcpy(&rtmsg->rtmsg_metric, RTA_DATA(rta[RTA_PRIORITY-1]), 4);
	}
	return 0;
}

int inet6_rtm_delroute(struct sk_buff *skb, struct nlmsghdr* nlh, void *arg)
{
	struct rtmsg *r = NLMSG_DATA(nlh);
	struct in6_rtmsg rtmsg;

	if (inet6_rtm_to_rtmsg(r, arg, &rtmsg))
		return -EINVAL;
	return ip6_tb_route_del(r->rtm_table, &rtmsg, nlh, arg, &NETLINK_CB(skb));
}

int inet6_rtm_newroute(struct sk_buff *skb, struct nlmsghdr* nlh, void *arg)
{
	struct rtmsg *r = NLMSG_DATA(nlh);
	struct in6_rtmsg rtmsg;

	if (inet6_rtm_to_rtmsg(r, arg, &rtmsg))
		return -EINVAL;
	return ip6_tb_route_add(r->rtm_table, &rtmsg, nlh, arg, &NETLINK_CB(skb));
}

struct rt6_rtnl_dump_arg
{
	struct sk_buff *skb;
	struct netlink_callback *cb;
};

static int rt6_fill_node(struct sk_buff *skb, struct rt6_info *rt,
			 struct in6_addr *dst, struct in6_addr *src,
			 int iif, int type, u32 pid, u32 seq,
			 int prefix, unsigned int flags)
{
	struct rtmsg *rtm;
	struct nlmsghdr  *nlh;
	unsigned char	 *b = skb->tail;
	struct rta_cacheinfo ci;
	__u16 nlmsg_flags = 0;

	if (prefix) {	/* user wants prefix routes only */
		if (!(rt->rt6i_flags & RTF_PREFIX_RT)) {
			/* success since this is not a prefix route */
			return 1;
		}
	}

	if (in_nlh) {
		if (in_nlh->nlmsg_flags&NLM_F_DUMP)
			nlmsg_flags = NLM_F_MULTI;
	}

	nlh = NLMSG_NEW(skb, pid, seq, type, sizeof(*rtm), flags);
	nlh->nlmsg_flags |= nlmsg_flags;
	rtm = NLMSG_DATA(nlh);
	rtm->rtm_family = AF_INET6;
	rtm->rtm_dst_len = rt->rt6i_dst.plen;
	rtm->rtm_src_len = rt->rt6i_src.plen;
	rtm->rtm_tos = 0;
	rtm->rtm_table = rt->rt6i_table;
	if (rt->rt6i_flags&RTF_REJECT)
		rtm->rtm_type = RTN_UNREACHABLE;
	else if (rt->rt6i_dev && (rt->rt6i_dev->flags&IFF_LOOPBACK))
		rtm->rtm_type = RTN_LOCAL;
	else
		rtm->rtm_type = RTN_UNICAST;
	rtm->rtm_flags = 0;
	rtm->rtm_scope = RT_SCOPE_UNIVERSE;
	rtm->rtm_protocol = rt->rt6i_protocol;
	if (rt->rt6i_flags&RTF_DYNAMIC)
		rtm->rtm_protocol = RTPROT_REDIRECT;
	else if (rt->rt6i_flags & RTF_ADDRCONF)
		rtm->rtm_protocol = RTPROT_KERNEL;
	else if (rt->rt6i_flags&RTF_DEFAULT)
		rtm->rtm_protocol = RTPROT_RA;

	if (rt->rt6i_flags&RTF_CACHE)
		rtm->rtm_flags |= RTM_F_CLONED;

	if (rt->rt6i_flags&RTF_PREFIX_RT)
		rtm->rtm_flags |= RTM_F_PREFIX;

	if (dst) {
		RTA_PUT(skb, RTA_DST, 16, dst);
	        rtm->rtm_dst_len = 128;
	} else if (rtm->rtm_dst_len)
		RTA_PUT(skb, RTA_DST, 16, &rt->rt6i_dst.addr);
#ifdef CONFIG_IPV6_SUBTREES
	if (src) {
		RTA_PUT(skb, RTA_SRC, 16, src);
	        rtm->rtm_src_len = 128;
	} else if (rtm->rtm_src_len)
		RTA_PUT(skb, RTA_SRC, 16, &rt->rt6i_src.addr);
#endif
	if (iif)
		RTA_PUT(skb, RTA_IIF, 4, &iif);
	else if (dst) {
		struct in6_addr saddr_buf;
		if (ipv6_get_saddr(&rt->u.dst, dst, &saddr_buf) == 0)
			RTA_PUT(skb, RTA_PREFSRC, 16, &saddr_buf);
	}
	if (rtnetlink_put_metrics(skb, rt->u.dst.metrics) < 0)
		goto rtattr_failure;
	if (rt->u.dst.neighbour)
		RTA_PUT(skb, RTA_GATEWAY, 16, &rt->u.dst.neighbour->primary_key);
	if (rt->u.dst.dev)
		RTA_PUT(skb, RTA_OIF, sizeof(int), &rt->rt6i_dev->ifindex);
	RTA_PUT(skb, RTA_PRIORITY, 4, &rt->rt6i_metric);
	ci.rta_lastuse = jiffies_to_clock_t(jiffies - rt->u.dst.lastuse);
	if (rt->rt6i_expires)
		ci.rta_expires = jiffies_to_clock_t(rt->rt6i_expires - jiffies);
	else
		ci.rta_expires = 0;
	ci.rta_used = rt->u.dst.__use;
	ci.rta_clntref = atomic_read(&rt->u.dst.__refcnt);
	ci.rta_error = rt->u.dst.error;
	ci.rta_id = 0;
	ci.rta_ts = 0;
	ci.rta_tsage = 0;
	RTA_PUT(skb, RTA_CACHEINFO, sizeof(ci), &ci);
	nlh->nlmsg_len = skb->tail - b;
	return skb->len;

nlmsg_failure:
rtattr_failure:
	skb_trim(skb, b - skb->data);
	return -1;
}

static int rt6_dump_route(struct rt6_info *rt, void *p_arg)
{
	struct rt6_rtnl_dump_arg *arg = (struct rt6_rtnl_dump_arg *) p_arg;
	int prefix;

	if (arg->cb->nlh->nlmsg_len >= NLMSG_LENGTH(sizeof(struct rtmsg))) {
		struct rtmsg *rtm = NLMSG_DATA(arg->cb->nlh);
		prefix = (rtm->rtm_flags & RTM_F_PREFIX) != 0;
	} else
		prefix = 0;

	return rt6_fill_node(arg->skb, rt, NULL, NULL, 0, RTM_NEWROUTE,
		     NETLINK_CB(arg->cb->skb).pid, arg->cb->nlh->nlmsg_seq,
		     prefix, NLM_F_MULTI);
}

static int fib6_dump_node(struct fib6_walker_t *w)
{
	int res;
	struct rt6_info *rt;

	for (rt = w->leaf; rt; rt = rt->u.next) {
		res = rt6_dump_route(rt, w->args);
		if (res < 0) {
			/* Frame is full, suspend walking */
			w->leaf = rt;
			return 1;
		}
		BUG_TRAP(res!=0);
	}
	w->leaf = NULL;
	return 0;
}

static void fib6_dump_end(struct netlink_callback *cb)
{
	struct fib6_walker_t *w = (void*)cb->args[0];

	if (w) {
		cb->args[0] = 0;
		kfree(w);
	}
	if (cb->args[1]) {
		cb->done = (void*)cb->args[1];
		cb->args[1] = 0;
	}
}

static int fib6_dump_done(struct netlink_callback *cb)
{
	fib6_dump_end(cb);
	return cb->done(cb);
}

int inet6_dump_fib(struct sk_buff *skb, struct netlink_callback *cb)
{
	struct rt6_rtnl_dump_arg arg;
	struct fib6_walker_t *w;
	struct rt6_table *table;
	int res = 0;
	int i;

	arg.skb = skb;
	arg.cb = cb;

	w = (void*)cb->args[0];
	if (w == NULL) {
		i = RT6_TABLE_MIN;
		/* New dump:
		 * 
		 * 1. hook callback destructor.
		 */
		cb->args[1] = (long)cb->done;
		cb->done = fib6_dump_done;

		/*
		 * 2. allocate and initialize walker.
		 */
		w = kmalloc(sizeof(*w), GFP_ATOMIC);
		if (w == NULL)
			return -ENOMEM;
		RT6_TRACE("dump<%p", w);
		memset(w, 0, sizeof(*w));
		w->func = fib6_dump_node;
		w->args = &arg;
		cb->args[0] = (long)w;
	restart_dump_iterator:
		read_lock_bh(&rt6_lock);
		while (i <= RT6_TABLE_MAX) { 
			if ((table = rt6_get_table(i))) {
				read_lock(&table->lock);
				w->root = &table->root;
				res = fib6_walk(w);
				read_unlock(&table->lock);
				if (res)
					break;
			}
			i++;
		}
		cb->args[2] = i;
		read_unlock_bh(&rt6_lock);
	} else {
		i = cb->args[2];

		if (i > RT6_TABLE_MAX)
			goto out;

		w->args = &arg;
			
		read_lock_bh(&rt6_lock);
		table = rt6_get_table(i);
		if (likely(table != NULL)) {
			read_lock(&table->lock);
			res = fib6_walk_continue(w);
			read_unlock(&table->lock);
			if (res) {
				if (res < 0)
					fib6_walker_unlink(w);
				read_unlock_bh(&rt6_lock);
				goto out;
			}
		}
		fib6_walker_unlink(w);
		read_unlock_bh(&rt6_lock);
		i++;
		goto restart_dump_iterator;
	}
out:
#if RT6_DEBUG >= 3
	if (res <= 0 && skb->len == 0)
		RT6_TRACE("%p>dump end\n", w);
#endif
	res = res < 0 ? res : skb->len;
	/* res < 0 is an error. (really, impossible)
	   res == 0 means that dump is complete, but skb still can contain data.
	   res > 0 dump is not complete, but frame is full.
	 */
	/* Destroy walker, if dump of this table is complete. */
	if (res <= 0)
		fib6_dump_end(cb);
	return res;
}

int inet6_rtm_getroute(struct sk_buff *in_skb, struct nlmsghdr* nlh, void *arg)
{
	struct rtattr **rta = arg;
	int iif = 0;
	int err = -ENOBUFS;
	struct sk_buff *skb;
	struct flowi fl;
	struct rt6_info *rt;

	skb = alloc_skb(NLMSG_GOODSIZE, GFP_KERNEL);
	if (skb == NULL)
		goto out;

	/* Reserve room for dummy headers, this skb can pass
	   through good chunk of routing engine.
	 */
	skb->mac.raw = skb->data;
	skb_reserve(skb, MAX_HEADER + sizeof(struct ipv6hdr));

	memset(&fl, 0, sizeof(fl));
	if (rta[RTA_SRC-1])
		ipv6_addr_copy(&fl.fl6_src,
			       (struct in6_addr*)RTA_DATA(rta[RTA_SRC-1]));
	if (rta[RTA_DST-1])
		ipv6_addr_copy(&fl.fl6_dst,
			       (struct in6_addr*)RTA_DATA(rta[RTA_DST-1]));

	if (rta[RTA_IIF-1])
		memcpy(&iif, RTA_DATA(rta[RTA_IIF-1]), sizeof(int));

	if (iif) {
		struct net_device *dev;
		dev = __dev_get_by_index(iif);
		if (!dev) {
			err = -ENODEV;
			goto out_free;
		}
		fl.iif = iif;
	} else
		fl.iif = loopback_dev.ifindex;

	fl.oif = 0;
	if (rta[RTA_OIF-1])
		memcpy(&fl.oif, RTA_DATA(rta[RTA_OIF-1]), sizeof(int));

	rt = (struct rt6_info*)ip6_route_output(NULL, &fl);

	skb->dst = &rt->u.dst;

	NETLINK_CB(skb).dst_pid = NETLINK_CB(in_skb).pid;
	err = rt6_fill_node(skb, rt, 
			    &fl.fl6_dst, &fl.fl6_src,
			    iif,
			    RTM_NEWROUTE, NETLINK_CB(in_skb).pid,
			    nlh->nlmsg_seq, 0, 0);
	if (err < 0) {
		err = -EMSGSIZE;
		goto out_free;
	}

	err = netlink_unicast(rtnl, skb, NETLINK_CB(in_skb).pid, MSG_DONTWAIT);
	if (err > 0)
		err = 0;
out:
	return err;
out_free:
	kfree_skb(skb);
	goto out;	
}

void inet6_rt_notify(int event, struct rt6_info *rt, struct nlmsghdr *nlh, 
			struct netlink_skb_parms *req)
{
	struct sk_buff *skb;
	int size = NLMSG_SPACE(sizeof(struct rtmsg)+256);
	u32 pid = current->pid;
	u32 seq = 0;

	if (req)
		pid = req->pid;
	if (nlh)
		seq = nlh->nlmsg_seq;
	
	skb = alloc_skb(size, gfp_any());
	if (!skb) {
		netlink_set_err(rtnl, 0, RTMGRP_IPV6_ROUTE, ENOBUFS);
		return;
	}
	if (rt6_fill_node(skb, rt, NULL, NULL, 0, event, pid, seq, 0, 0) < 0) {
		kfree_skb(skb);
		netlink_set_err(rtnl, 0, RTMGRP_IPV6_ROUTE, EINVAL);
		return;
	}
	NETLINK_CB(skb).dst_groups = RTMGRP_IPV6_ROUTE;
	netlink_broadcast(rtnl, skb, 0, RTMGRP_IPV6_ROUTE, gfp_any());
}

/*
 *	/proc
 */

#ifdef CONFIG_PROC_FS

#define RT6_INFO_LEN (32 + 4 + 32 + 4 + 32 + 40 + 5 + 1)

struct rt6_proc_arg
{
	char *buffer;
	int offset;
	int length;
	int skip;
	int len;
};

static int rt6_info_route(struct rt6_info *rt, void *p_arg)
{
	struct rt6_proc_arg *arg = (struct rt6_proc_arg *) p_arg;
	int i;

	if (arg->skip < arg->offset / RT6_INFO_LEN) {
		arg->skip++;
		return 0;
	}

	if (arg->len >= arg->length)
		return 0;

	for (i=0; i<16; i++) {
		sprintf(arg->buffer + arg->len, "%02x",
			rt->rt6i_dst.addr.s6_addr[i]);
		arg->len += 2;
	}
	arg->len += sprintf(arg->buffer + arg->len, " %02x ",
			    rt->rt6i_dst.plen);

#ifdef CONFIG_IPV6_SUBTREES
	for (i=0; i<16; i++) {
		sprintf(arg->buffer + arg->len, "%02x",
			rt->rt6i_src.addr.s6_addr[i]);
		arg->len += 2;
	}
	arg->len += sprintf(arg->buffer + arg->len, " %02x ",
			    rt->rt6i_src.plen);
#else
	sprintf(arg->buffer + arg->len,
		"00000000000000000000000000000000 00 ");
	arg->len += 36;
#endif

	if (rt->rt6i_nexthop) {
		for (i=0; i<16; i++) {
			sprintf(arg->buffer + arg->len, "%02x",
				rt->rt6i_nexthop->primary_key[i]);
			arg->len += 2;
		}
	} else {
		sprintf(arg->buffer + arg->len,
			"00000000000000000000000000000000");
		arg->len += 32;
	}
	arg->len += sprintf(arg->buffer + arg->len,
			    " %08x %08x %08x %08x %8s\n",
			    rt->rt6i_metric, atomic_read(&rt->u.dst.__refcnt),
			    rt->u.dst.__use, rt->rt6i_flags, 
			    rt->rt6i_dev ? rt->rt6i_dev->name : "");
	return 0;
}

static int rt6_proc_info(char *buffer, char **start, off_t offset, int length)
{
	struct rt6_proc_arg arg;
	int i;	
	arg.buffer = buffer;
	arg.offset = offset;
	arg.length = length;
	arg.skip = 0;
	arg.len = 0;


	read_lock_bh(&rt6_lock);
	for (i = RT6_TABLE_MIN; i <= RT6_TABLE_MAX; i++) {
		struct rt6_table *table = rt6_get_table(i);
		if (table) {
			read_lock(&table->lock);
			fib6_clean_tree(&table->root, rt6_info_route, 0, &arg);
			read_unlock(&table->lock);
		}
	}
	read_unlock_bh(&rt6_lock);

	*start = buffer;
	if (offset)
		*start += offset % RT6_INFO_LEN;

	arg.len -= offset % RT6_INFO_LEN;

	if (arg.len > length)
		arg.len = length;
	if (arg.len < 0)
		arg.len = 0;

	return arg.len;
}

extern struct rt6_statistics rt6_stats;

static int rt6_stats_seq_show(struct seq_file *seq, void *v)
{
	seq_printf(seq, "%04x %04x %04x %04x %04x %04x %04x\n",
		      rt6_stats.fib_nodes, rt6_stats.fib_route_nodes,
		      rt6_stats.fib_rt_alloc, rt6_stats.fib_rt_entries,
		      rt6_stats.fib_rt_cache,
		      atomic_read(&ip6_dst_ops.entries),
		      rt6_stats.fib_discarded_routes);

	return 0;
}

static int rt6_stats_seq_open(struct inode *inode, struct file *file)
{
	return single_open(file, rt6_stats_seq_show, NULL);
}

static struct file_operations rt6_stats_seq_fops = {
	.owner	 = THIS_MODULE,
	.open	 = rt6_stats_seq_open,
	.read	 = seq_read,
	.llseek	 = seq_lseek,
	.release = single_release,
};
#endif	/* CONFIG_PROC_FS */

#ifdef CONFIG_SYSCTL

static int flush_delay;

static
int ipv6_sysctl_rtcache_flush(ctl_table *ctl, int write, struct file * filp,
			      void __user *buffer, size_t *lenp, loff_t *ppos)
{
	if (write) {
		proc_dointvec(ctl, write, filp, buffer, lenp, ppos);
		fib6_run_gc(flush_delay <= 0 ? ~0UL : (unsigned long)flush_delay);
		return 0;
	} else
		return -EINVAL;
}

ctl_table ipv6_route_table[] = {
        {
		.ctl_name	=	NET_IPV6_ROUTE_FLUSH, 
		.procname	=	"flush",
         	.data		=	&flush_delay,
		.maxlen		=	sizeof(int),
		.mode		=	0200,
         	.proc_handler	=	&ipv6_sysctl_rtcache_flush
	},
	{
		.ctl_name	=	NET_IPV6_ROUTE_GC_THRESH,
		.procname	=	"gc_thresh",
         	.data		=	&ip6_dst_ops.gc_thresh,
		.maxlen		=	sizeof(int),
		.mode		=	0644,
         	.proc_handler	=	&proc_dointvec,
	},
	{
		.ctl_name	=	NET_IPV6_ROUTE_MAX_SIZE,
		.procname	=	"max_size",
         	.data		=	&ip6_rt_max_size,
		.maxlen		=	sizeof(int),
		.mode		=	0644,
         	.proc_handler	=	&proc_dointvec,
	},
	{
		.ctl_name	=	NET_IPV6_ROUTE_GC_MIN_INTERVAL,
		.procname	=	"gc_min_interval",
         	.data		=	&ip6_rt_gc_min_interval,
		.maxlen		=	sizeof(int),
		.mode		=	0644,
         	.proc_handler	=	&proc_dointvec_jiffies,
		.strategy	=	&sysctl_jiffies,
	},
	{
		.ctl_name	=	NET_IPV6_ROUTE_GC_TIMEOUT,
		.procname	=	"gc_timeout",
         	.data		=	&ip6_rt_gc_timeout,
		.maxlen		=	sizeof(int),
		.mode		=	0644,
         	.proc_handler	=	&proc_dointvec_jiffies,
		.strategy	=	&sysctl_jiffies,
	},
	{
		.ctl_name	=	NET_IPV6_ROUTE_GC_INTERVAL,
		.procname	=	"gc_interval",
         	.data		=	&ip6_rt_gc_interval,
		.maxlen		=	sizeof(int),
		.mode		=	0644,
         	.proc_handler	=	&proc_dointvec_jiffies,
		.strategy	=	&sysctl_jiffies,
	},
	{
		.ctl_name	=	NET_IPV6_ROUTE_GC_ELASTICITY,
		.procname	=	"gc_elasticity",
         	.data		=	&ip6_rt_gc_elasticity,
		.maxlen		=	sizeof(int),
		.mode		=	0644,
         	.proc_handler	=	&proc_dointvec_jiffies,
		.strategy	=	&sysctl_jiffies,
	},
	{
		.ctl_name	=	NET_IPV6_ROUTE_MTU_EXPIRES,
		.procname	=	"mtu_expires",
         	.data		=	&ip6_rt_mtu_expires,
		.maxlen		=	sizeof(int),
		.mode		=	0644,
         	.proc_handler	=	&proc_dointvec_jiffies,
		.strategy	=	&sysctl_jiffies,
	},
	{
		.ctl_name	=	NET_IPV6_ROUTE_MIN_ADVMSS,
		.procname	=	"min_adv_mss",
         	.data		=	&ip6_rt_min_advmss,
		.maxlen		=	sizeof(int),
		.mode		=	0644,
         	.proc_handler	=	&proc_dointvec_jiffies,
		.strategy	=	&sysctl_jiffies,
	},
	{
		.ctl_name	=	NET_IPV6_ROUTE_GC_MIN_INTERVAL_MS,
		.procname	=	"gc_min_interval_ms",
         	.data		=	&ip6_rt_gc_min_interval,
		.maxlen		=	sizeof(int),
		.mode		=	0644,
         	.proc_handler	=	&proc_dointvec_ms_jiffies,
		.strategy	=	&sysctl_ms_jiffies,
	},
	{ .ctl_name = 0 }
};

#endif

#ifdef CONFIG_IPV6_MULTIPLE_TABLES

static inline int rt6_tables_init(void)
{
	int err = 0;

	write_lock_bh(&rt6_lock);
	if ((rt6_tables[RT6_TABLE_MAIN] = rt6_tree_init()) == NULL) {
		err = -ENOBUFS;
	} else if ((rt6_tables[RT6_TABLE_LOCAL] = rt6_tree_init()) == NULL) {
		kfree(rt6_tables[RT6_TABLE_MAIN]);
		err = -ENOBUFS;
	}
	write_unlock_bh(&rt6_lock);
	return err;
}

static inline void rt6_tables_cleanup(void)
{
	int i;
	write_lock_bh(&rt6_lock);	
	for (i = RT6_TABLE_MIN; i <= RT6_TABLE_MAX; i++)
		if (rt6_tables[i]) {
			BUG_TRAP(rt6_tables[i]->root.leaf == &ip6_null_entry);
			kfree(rt6_tables[i]);
			rt6_tables[i] = NULL;
		}
	write_unlock_bh(&rt6_lock);	
}

#else

static inline int rt6_tables_init(void) 
{
	return 0;
}

static inline void rt6_tables_cleanup(void) {}

#endif


int __init ip6_route_init(void)
{
	int err;
	struct proc_dir_entry *p;

	if ((err = rt6_tables_init()) < 0)
		return err;

	ip6_dst_ops.kmem_cachep = kmem_cache_create("ip6_dst_cache",
						     sizeof(struct rt6_info),
						     0, SLAB_HWCACHE_ALIGN,
						     NULL, NULL);
	if (!ip6_dst_ops.kmem_cachep)
		panic("cannot create ip6_dst_cache");

	fib6_init();
#ifdef 	CONFIG_PROC_FS
	p = proc_net_create("ipv6_route", 0, rt6_proc_info);
	if (p)
		p->owner = THIS_MODULE;

	proc_net_fops_create("rt6_stats", S_IRUGO, &rt6_stats_seq_fops);
#endif
#ifdef CONFIG_XFRM
	xfrm6_init();
#endif
	return 0;
}

void ip6_route_cleanup(void)
{
#ifdef CONFIG_PROC_FS
	proc_net_remove("ipv6_route");
	proc_net_remove("rt6_stats");
#endif
#ifdef CONFIG_XFRM
	xfrm6_fini();
#endif
	rt6_ifdown(NULL);
	rt6_tables_cleanup();
	fib6_gc_cleanup();
	kmem_cache_destroy(ip6_dst_ops.kmem_cachep);
}
