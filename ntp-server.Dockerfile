FROM alpine:latest

# install chrony
RUN apk add --no-cache chrony tzdata \
    && mkdir -p /run/chrony \
    && chown -R chrony:chrony /run/chrony \
    && chmod o-rx /run/chrony \
    && chown -R chrony:chrony /var/lib/chrony

# script to configure/startup chrony (ntp)
COPY ntp-server/chrony.conf /etc/chrony/chrony.conf

# ntp port
EXPOSE 123/udp

# let docker know how to test container health
HEALTHCHECK CMD chronyc -n tracking || exit 1

# start chronyd in the foreground
CMD [ "/usr/sbin/chronyd", "-u", "chrony", "-d", "-x", "-L", "0" ]
